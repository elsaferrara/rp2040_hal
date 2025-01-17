--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--

with RP2040_SVD.SIO; use RP2040_SVD.SIO;
with RP.Reset;

package body RP.GPIO with SPARK_Mode is
   function Pin_Mask (Pin : GPIO_Pin)
      return GPIO_Pin_Mask
   is (GPIO_Pin_Mask (2 ** GPIO_Pin'Pos (Pin)));

   procedure Enable is
      use RP.Reset;
   begin
      Reset_Peripheral (Reset_IO_BANK0);
      Reset_Peripheral (Reset_PADS_BANK0);

      --  Errata RP2040-E6
      --
      --  GPIO26-29 are shared with ADC inputs AIN0-3. The GPIO digital input
      --  is enabled after RUN is released. If the pins are connected to an
      --  analogue signal to measure, there could be unexpected signal levels
      --  on these pads. This is unlikely to cause a problem as the digital
      --  inputs have hysteresis enabled by default.

      for Pin in ADC_Pin'Range loop
         PADS_BANK_Periph.GPIO (Pin).IE := False;
         PADS_BANK_Periph.GPIO (Pin).OD := True;
      end loop;

      --  Mask all pin interrupts
      IO_BANK_Periph.PROC0_INTE := (others => (others => 0));
      IO_BANK_Periph.PROC1_INTE := (others => (others => 0));

      GPIO_Enabled := True;
   end Enable;

   function Enabled
      return Boolean
   is (GPIO_Enabled);

   procedure Configure
      (This      : GPIO_Point;
       Mode      : GPIO_Config_Mode;
       Pull      : GPIO_Pull_Mode := Floating;
       Func      : GPIO_Function := SIO;
       Schmitt   : Boolean := False;
       Slew_Fast : Boolean := False;
       Drive     : GPIO_Drive := Drive_4mA)
   is
      Mask : constant GPIO_Pin_Mask := Pin_Mask (This.Pin);
   begin
      if not Enabled then
         Enable;
      end if;

      IO_BANK_Periph.GPIO (This.Pin).CTRL :=
         (FUNCSEL => Func,
          others  => <>);

      case Mode is
         when Input =>
            PADS_BANK_Periph.GPIO (This.Pin) :=
               (PUE      => (Pull = Pull_Both or Pull = Pull_Up),
                PDE      => (Pull = Pull_Both or Pull = Pull_Down),
                IE       => True,
                OD       => True,
                SCHMITT  => Schmitt,
                SLEWFAST => Slew_Fast,
                DRIVE    => GPIO0_DRIVE_Field'Val (GPIO_Drive'Pos (Drive)),
                others   => <>);
         when Output =>
            PADS_BANK_Periph.GPIO (This.Pin) :=
               (PUE      => (Pull = Pull_Both or Pull = Pull_Up),
                PDE      => (Pull = Pull_Both or Pull = Pull_Down),
                IE       => True,
                OD       => False,
                SCHMITT  => Schmitt,
                SLEWFAST => Slew_Fast,
                DRIVE    => GPIO0_DRIVE_Field'Val (GPIO_Drive'Pos (Drive)),
                others   => <>);
            SIO_Periph.GPIO_OUT_CLR.GPIO_OUT_CLR := Mask;
            SIO_Periph.GPIO_OE_SET.GPIO_OE_SET := Mask;
         when Analog =>
            PADS_BANK_Periph.GPIO (This.Pin) :=
               (PUE      => False,
                PDE      => False,
                IE       => True,
                OD       => True,
                SCHMITT  => Schmitt,
                SLEWFAST => Slew_Fast,
                DRIVE    => GPIO0_DRIVE_Field'Val (GPIO_Drive'Pos (Drive)),
                others   => <>);
            IO_BANK_Periph.GPIO (This.Pin).CTRL.FUNCSEL := HI_Z;
      end case;
   end Configure;

   procedure Get
     (This : GPIO_Point;
       Result : out Boolean)
   is
      Tmp : constant UInt30 := SIO_Periph.GPIO_IN.GPIO_IN;
   begin
      Result := ((Tmp and Pin_Mask (This.Pin)) /= 0);
   end Get;

   procedure Enable_Interrupt
     (This    : GPIO_Point;
      Trigger : Interrupt_Triggers)
     with SPARK_Mode => Off
   is
      Group  : constant Natural := Natural (This.Pin) / 8;
      Offset : constant Natural := Natural (This.Pin) mod 8;
      Mask   : constant UInt4 := Interrupt_Triggers'Enum_Rep (Trigger);
      Tmp   : constant UInt4 := IO_BANK_Periph.PROC0_INTE (Group) (Offset);
   begin
      IO_BANK_Periph.INTR (Group) (Offset) := Mask;
      IO_BANK_Periph.PROC0_INTE (Group) (Offset) := Tmp or Mask;
   end Enable_Interrupt;

   procedure Disable_Interrupt
     (This    : GPIO_Point;
      Trigger : Interrupt_Triggers)
     with SPARK_Mode => Off
   is
      Group  : constant Natural := Natural (This.Pin) / 8;
      Offset : constant Natural := Natural (This.Pin) mod 8;
      Mask   : constant UInt4 := Interrupt_Triggers'Enum_Rep (Trigger);
      Tmp   : constant UInt4 := IO_BANK_Periph.PROC0_INTE (Group) (Offset);
   begin
      IO_BANK_Periph.INTR (Group) (Offset) := Mask;
      IO_BANK_Periph.PROC0_INTE (Group) (Offset) := Tmp and not Mask;
   end Disable_Interrupt;

   procedure Acknowledge_Interrupt
     (Pin     : GPIO_Pin;
      Trigger : Interrupt_Triggers)
     with SPARK_Mode => Off
   is
      Group  : constant Natural := Natural (Pin) / 8;
      Offset : constant Natural := Natural (Pin) mod 8;
      Mask   : constant UInt4 := Interrupt_Triggers'Enum_Rep (Trigger);
   begin
      IO_BANK_Periph.INTR (Group) (Offset) := Mask;
   end Acknowledge_Interrupt;

   function Interrupt_Status
     (Pin     : GPIO_Pin;
      Trigger : Interrupt_Triggers)
       return Boolean
   is
      Group  : constant Natural := Natural (Pin) / 8;
      Offset : constant Natural := Natural (Pin) mod 8;
      Mask   : constant UInt4 := Interrupt_Triggers'Enum_Rep (Trigger);
      Tmp   : constant UInt4 := IO_BANK_Periph.PROC0_INTS (Group) (Offset);
   begin
      return (Tmp and Mask) /= 0;
   end Interrupt_Status;

   function Support
     --  (This : GPIO_Point;
     --   Capa : HAL.GPIO.Capability)
       return Boolean
   is (True);

   function Mode
     (This : GPIO_Point)
       return HAL.GPIO.GPIO_Mode
   is
      Tmp : constant GPIO_Function := IO_BANK_Periph.GPIO (This.Pin).CTRL.FUNCSEL;
      Tmp2 : constant Boolean := PADS_BANK_Periph.GPIO (This.Pin).OD;
   begin
      if Tmp /= SIO then
         return Unknown_Mode;
      elsif Tmp2 then
         return Input;
      else
         return Output;
      end if;
   end Mode;

   procedure Set_Mode
     (This : GPIO_Point;
      Mode : HAL.GPIO.GPIO_Config_Mode)
   is
   begin
      case Mode is
         when HAL.GPIO.Input =>
            Configure (This, Input);
         when HAL.GPIO.Output =>
            Configure (This, Output);
      end case;
   end Set_Mode;

   function Pull_Resistor
     (This : GPIO_Point)
       return HAL.GPIO.GPIO_Pull_Resistor
   is
      Tmp : constant Boolean := PADS_BANK_Periph.GPIO (This.Pin).PUE;
      Tmp2 : constant Boolean := PADS_BANK_Periph.GPIO (This.Pin).PDE;
   begin
      if Tmp then
         return Pull_Up;
      elsif Tmp2 then
         return Pull_Down;
      else
         return Floating;
      end if;
   end Pull_Resistor;

   procedure Set_Pull_Resistor
     (This : GPIO_Point;
      Pull : HAL.GPIO.GPIO_Pull_Resistor)
   is
   begin
      case Pull is
         when Pull_Up =>
            PADS_BANK_Periph.GPIO (This.Pin).PUE := True;
            PADS_BANK_Periph.GPIO (This.Pin).PDE := False;
         when Pull_Down =>
            PADS_BANK_Periph.GPIO (This.Pin).PUE := False;
            PADS_BANK_Periph.GPIO (This.Pin).PDE := True;
         when Floating =>
            PADS_BANK_Periph.GPIO (This.Pin).PUE := False;
            PADS_BANK_Periph.GPIO (This.Pin).PDE := False;
      end case;
   end Set_Pull_Resistor;

   procedure Set
     (This : GPIO_Point;
      Result : out Boolean)
   is
      Tmp : constant UInt30 := SIO_Periph.GPIO_IN.GPIO_IN;
   begin
      Result := ((Tmp and Pin_Mask (This.Pin)) /= 0);
   end Set;

   procedure Set
     (This : GPIO_Point)
   is
   begin
      SIO_Periph.GPIO_OUT_SET.GPIO_OUT_SET := Pin_Mask (This.Pin);
   end Set;

   procedure Clear
     (This : GPIO_Point)
   is
   begin
      SIO_Periph.GPIO_OUT_CLR.GPIO_OUT_CLR := Pin_Mask (This.Pin);
   end Clear;

   procedure Toggle
     (This : GPIO_Point)
   is
   begin
      SIO_Periph.GPIO_OUT_XOR.GPIO_OUT_XOR := Pin_Mask (This.Pin);
   end Toggle;

end RP.GPIO;
