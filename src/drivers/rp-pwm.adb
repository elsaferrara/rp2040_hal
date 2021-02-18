--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP2040_SVD.RESETS; use RP2040_SVD.RESETS;
with RP.Clock;

package body RP.PWM is
   function To_PWM
      (GPIO : RP.GPIO.GPIO_Point)
      return PWM_Point
   is ((Slice   => PWM_Slice (Shift_Right (UInt32 (GPIO.Pin), 1) and 2#111#),
        Channel => PWM_Channel'Val (UInt32 (GPIO.Pin) and 1)));

   procedure Enable
      (Slice : PWM_Slice)
   is
   begin
      if not RESETS_Periph.RESET_DONE.pwm then
         RESETS_Periph.RESET.pwm := False;
         while not RESETS_Periph.RESET_DONE.pwm loop
            null;
         end loop;
      end if;

      PWM_Periph.EN.CH.Arr (Natural (Slice)) := True;
   end Enable;

   procedure Disable
      (Slice : PWM_Slice)
   is
   begin
      PWM_Periph.EN.CH.Arr (Natural (Slice)) := False;
   end Disable;

   function Enabled
      (Slice : PWM_Slice)
      return Boolean
   is (PWM_Periph.EN.CH.Arr (Natural (Slice)));

   procedure Set_Mode
      (Slice : PWM_Slice;
       Mode  : PWM_Divider_Mode)
   is
      use RP2040_SVD.PWM;
   begin
      PWM_Periph.CH (Slice).CSR.DIVMODE :=
         CH0_CSR_DIVMODE_Field'Enum_Val (PWM_Divider_Mode'Enum_Rep (Mode));
   end Set_Mode;

   procedure Set_Interval
      (Slice  : PWM_Slice;
       Clocks : Period)
   is
   begin
      PWM_Periph.CH (Slice).TOP.CH0_TOP := Clocks;
   end Set_Interval;

   procedure Set_Duty_Cycle
      (Point  : PWM_Point;
       Clocks : Period)
   is
   begin
      if Point.Channel = A then
         PWM_Periph.CH (Point.Slice).CC.A := Clocks;
      else
         PWM_Periph.CH (Point.Slice).CC.B := Clocks;
      end if;
   end Set_Duty_Cycle;

   procedure Set_Invert
      (Point  : PWM_Point;
       Invert : Boolean)
   is
   begin
      if Point.Channel = A then
         PWM_Periph.CH (Point.Slice).CSR.A_INV := Invert;
      else
         PWM_Periph.CH (Point.Slice).CSR.B_INV := Invert;
      end if;
   end Set_Invert;

   procedure Set_Phase_Correction
      (Slice   : PWM_Slice;
       Enabled : Boolean)
   is
   begin
      PWM_Periph.CH (Slice).CSR.PH_CORRECT := Enabled;
   end Set_Phase_Correction;

   procedure Advance_Phase
      (Slice : PWM_Slice)
   is
   begin
      PWM_Periph.CH (Slice).CSR.PH_ADV := True;
      while PWM_Periph.CH (Slice).CSR.PH_ADV loop
         null;
      end loop;
   end Advance_Phase;

   procedure Retard_Phase
      (Slice : PWM_Slice)
   is
   begin
      PWM_Periph.CH (Slice).CSR.PH_RET := True;
      while PWM_Periph.CH (Slice).CSR.PH_RET loop
         null;
      end loop;
   end Retard_Phase;

   procedure Set_Divider
      (Slice : PWM_Slice;
       Div   : Divider)
   is
      use RP2040_SVD.PWM;
   begin
      PWM_Periph.CH (Slice).DIV :=
         (INT    => CH0_DIV_INT_Field  (Div_Integer (Div)),
          FRAC   => CH0_DIV_FRAC_Field (Div_Fraction (Div)),
          others => <>);
   end Set_Divider;

   procedure Set_Frequency
      (Slice     : PWM_Slice;
       Frequency : Hertz)
   is
   begin
      Set_Divider (Slice, Divider (RP.Clock.Frequency (RP.Clock.SYS) / Frequency));
   end Set_Frequency;

   function Count
      (Slice : PWM_Slice)
      return Natural
   is (Natural (PWM_Periph.CH (Slice).CTR.CH0_CTR));

   function Div_Integer
      (V : Divider)
      return UInt8
   is
      I : constant Natural := Natural (V);
   begin
      if Divider (I) > V then
         return UInt8 (I - 1);
      else
         return UInt8 (I);
      end if;
   end Div_Integer;

   function Div_Fraction
      (V : Divider)
      return UInt4
   is (UInt4 ((V - Divider (Div_Integer (V))) * 2 ** 4));

   function Div_Value
      (Int  : UInt8;
       Frac : UInt4)
       return Divider
   is (Divider (Int) + (Divider (Frac) / 2 ** 4));

end RP.PWM;
