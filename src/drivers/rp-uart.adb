--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with HAL; use HAL;
with RP.Reset;
with RP.Timer;
with Ada.Unchecked_Conversion;

package body RP.UART with SPARK_Mode is

   procedure Configure
     (This   : in out UART_Port;
      Config : UART_Configuration := Default_UART_Configuration)
   is
      procedure Configure_Inner
        (This   : in out UART_Port;
         Config : UART_Configuration := Default_UART_Configuration;
         Periph : in out UART_Peripheral)
      is
         use RP.Reset;
         Word_Length : constant UInt2 := UInt2
           (Config.Word_Size - UART_Word_Size'First);
         Num : constant UART_Number := This.Num;
      begin
         case Num is
         when 0 => Reset_Peripheral (Reset_UART0);
         when 1 => Reset_Peripheral (Reset_UART1);
         end case;

         Periph.UARTDMACR :=
           (RXDMAE => True,
            TXDMAE => True,
            others => <>);

         declare
            Clock_Frequency : Hertz;
            Int : Natural;
            Remainder : Natural;
            Multiple : constant Natural := (2 ** UARTFBRD_BAUD_DIVFRAC_Field'Size);
         begin
            RP.Clock.Frequency (RP.Clock.PERI, Clock_Frequency);
            Int := Clock_Frequency / (Config.Baud * 16);
            Remainder := Clock_Frequency mod (Config.Baud * 16);
            pragma Assert (Int >= 0);
            pragma Assert (Int <= 2 ** 16 - 1);
            Periph.UARTIBRD.BAUD_DIVINT := UInt16 (Int);
            --  pragma Assert (Remainder < (Config.Baud * 16));
            --  pragma Assert (Multiple <= 2 ** 6);
            --  pragma Assert (Multiple /= 0);
            --  pragma Assert (Remainder / (Config.Baud * 16) < 1);
            --  pragma Assert (Remainder / (Config.Baud * 16) * Multiple < Multiple);
            --  pragma Assert (((Long_Long_Integer (Remainder) * Long_Long_Integer (Multiple)) / Long_Long_Integer (Config.Baud * 16)) <= Long_Long_Integer (Multiple));
            Periph.UARTFBRD.BAUD_DIVFRAC :=
              UInt6 ((Long_Long_Integer (Remainder) * Long_Long_Integer (Multiple))
                     / Long_Long_Integer (Config.Baud * 16));
         end;

         Periph.UARTLCR_H :=
           (WLEN   => Word_Length,
            PEN    => Config.Parity,
            EPS    => Config.Parity_Type = Even,
            STP2   => (Config.Stop_Bits = 2),
            SPS    => False, --  Stick parity is disabled by default
            FEN    => Config.Enable_FIFOs,
            BRK    => False, --  Don't send break initially
            others => <>);

         Periph.UARTCR :=
           (UARTEN => True,
            TXE    => True,
            RXE    => True,
            LBE    => Config.Loopback,
            others => <>);

         This.Config := Config;
      end Configure_Inner;
   begin
      case This.Num is
      when 0 => Configure_Inner (This, Config, UART0_Periph);
         when 1 => Configure_Inner (This, Config, UART1_Periph);
      end case;
   end Configure;

   procedure Set_Stick_Parity
     (This    : in UART_Port;
      Enabled : Boolean)
   is
      procedure Set_Stick_Parity_Inner
        (Enabled : Boolean;
         Periph : in out UART_Peripheral)
      is
      begin
         Periph.UARTLCR_H.SPS := Enabled;
      end Set_Stick_Parity_Inner;
   begin
      case This.Num is
      when 0 => Set_Stick_Parity_Inner (Enabled, UART0_Periph);
         when 1 => Set_Stick_Parity_Inner (Enabled, UART1_Periph);
      end case;
   end Set_Stick_Parity;

   function Symbol_Time
     (This : UART_Port)
       return Microseconds
   is
      Tmp : constant Natural := This.Config.Baud;
   begin
      return ((1_000_000 / Integer (Tmp)) + 1);
   end Symbol_Time;

   function Frame_Time
     (This : UART_Port)
       return Integer
   is
      Start_Bits  : constant Integer := 1;
      Word_Size : constant Integer := Integer (This.Config.Word_Size);
      Frame_Length : constant Integer := This.Config.Frame_Length;
      Data_Bits   : constant Integer := Word_Size * Frame_Length;
      Parity : constant Boolean := This.Config.Parity;
      Parity_Bits : constant Integer := (if Parity then 1 else 0);
      Stop_Bits   : constant Integer := Integer (This.Config.Stop_Bits);
      Frame_Bits  : constant Integer := Start_Bits + Data_Bits + Parity_Bits + Stop_Bits;
      Symbol_Time : constant Microseconds := This.Symbol_Time;
   begin
      return Frame_Bits * Symbol_Time;
   end Frame_Time;

   procedure Send_Break
     (This     : in out UART_Port;
      Delays   : not null HAL.Time.Any_Delays;
      Duration : Microseconds;
      Start    : Boolean := True)
   is
      procedure Send_Break_Inner
        (This     : UART_Port;
         Delays   : not null HAL.Time.Any_Delays;
         Duration : Microseconds;
         Start    : Boolean := True;
         Periph : in out UART_Peripheral)
      is
         Transmit_Status : UART_FIFO_Status;
         Symbol_Time : Microseconds;
      begin
         --  Wait for any in progress transmission to complete before setting up a break
         loop
            Transmit_Status := This.Transmit_Status;
            exit when Transmit_Status = Empty;
         end loop;

         if Start then
            Symbol_Time := This.Symbol_Time;
            Delays.Delay_Microseconds (Symbol_Time);
         end if;
         Periph.UARTLCR_H.BRK := True;
         Delays.Delay_Microseconds (Duration);
         Periph.UARTLCR_H.BRK := False;
      end Send_Break_Inner;
   begin
      case This.Num is
         when 0 => Send_Break_Inner (This, Delays, Duration, Start, UART0_Periph);
         when 1 => Send_Break_Inner (This, Delays, Duration, Start, UART1_Periph);
      end case;
   end Send_Break;

   function Transmit_Status
     (This : UART_Port)
      return UART_FIFO_Status
   is
      function Transmit_Status_Inner
        (Periph : UART_Peripheral)
         return UART_FIFO_Status
        with Volatile_Function;

      function Transmit_Status_Inner
        (Periph : UART_Peripheral)
         return UART_FIFO_Status
      is
         --  TXFE TXFF
         --  0    0     Not_Full
         --  0    1     Full
         --  1    0     Empty
         --  1    1     Invalid
         Is_TXFE : constant Boolean := Periph.UARTFR.TXFE;
         Is_TXFF : constant Boolean := Periph.UARTFR.TXFF;
         Is_BUSY : constant Boolean := Periph.UARTFR.BUSY;
      begin
         if not Is_TXFE and not Is_TXFF then
            return Not_Full;
         elsif not Is_TXFE and Is_TXFF then
            return Full;
         elsif Is_BUSY then
            return Busy;
         elsif Is_TXFE and not Is_TXFF then
            return Empty;
         else
            return Invalid;
         end if;
      end Transmit_Status_Inner;
      Result : UART_FIFO_Status;
   begin
      case This.Num is
         when 0 => Result := Transmit_Status_Inner (UART0_Periph);
         when 1 => Result :=  Transmit_Status_Inner (UART1_Periph);
      end case;
      return Result;
   end Transmit_Status;

   function Receive_Status
     (This : UART_Port)
      return UART_FIFO_Status
   is
      function Receive_Status_Inner
        (Periph : UART_Peripheral)
         return UART_FIFO_Status
        with Volatile_Function;

      function Receive_Status_Inner
        (Periph : UART_Peripheral)
         return UART_FIFO_Status
      is
         --  RXFE RXFF
         --  0    0     Not_Full
         --  0    1     Full
         --  1    0     Empty
         --  1    1     Invalid
         Flags : UARTFR_Register renames Periph.UARTFR;
         Is_RXFE : constant Boolean := Flags.RXFE;
         Is_RXFF : constant Boolean := Flags.RXFF;
      begin
         if not Is_RXFE and not Is_RXFF then
            return Not_Full;
         elsif not Is_RXFE and Is_RXFF then
            return Full;
         elsif Is_RXFE and not Is_RXFF then
            return Empty;
         else
            return Invalid;
         end if;
      end Receive_Status_Inner;
      Result : UART_FIFO_Status;
   begin
      case This.Num is
         when 0 => Result := Receive_Status_Inner (UART0_Periph);
         when 1 => Result := Receive_Status_Inner (UART1_Periph);
      end case;
      return Result;
   end Receive_Status;


   function FIFO_Address
     (This : UART_Port)
      return System.Address
     with SPARK_Mode => Off
   is
      function FIFO_Address_Inner
        (This : UART_Port;
         Periph : UART_Peripheral)
         return System.Address
      is
         UARTDR_Ptr : constant System.Address := Periph.UARTDR'Address;
      begin
         return UARTDR_Ptr;
      end FIFO_Address_Inner;
   begin
      case This.Num is
         when 0 => return FIFO_Address_Inner (This, UART0_Periph);
         when 1 => return FIFO_Address_Inner (This, UART1_Periph);
      end case;
   end FIFO_Address;

   --  overriding
   function Data_Size
     (Port : UART_Port)
      return UART_Data_Size
   is
      pragma Unreferenced (Port);
   begin
      return (Data_Size_8b);
   end Data_Size;

   --  overriding
   procedure Transmit
     (This    : in out UART_Port;
      Data    : UART_Data_8b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
   is
      procedure Transmit_Inner
        (This    : UART_Port;
         Data    : UART_Data_8b;
         Status  : out UART_Status;
         Timeout : Natural := 1000;
         Periph : in out UART_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : UART_FIFO_Status;
      begin
         if Timeout > 0 then
            Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
         end if;
         for D of Data loop
            loop
               FIFO := Transmit_Status (This);
               exit when FIFO = Empty or FIFO = Not_Full or FIFO = Busy;
               if FIFO = Invalid then
                  Status := Err_Error;
                  return;
               end if;
               if Timeout > 0 and then RP.Timer.Clock >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;

            Periph.UARTDR.DATA := D;
         end loop;
         Status := Ok;
      end Transmit_Inner;
   begin
      case This.Num is
         when 0 => Transmit_Inner (This, Data, Status, Timeout, UART0_Periph);
         when 1 => Transmit_Inner (This, Data, Status, Timeout, UART1_Periph);
      end case;
   end Transmit;

   DR       : UARTDR_Register;
   --  overriding
   procedure Receive
     (This    : in out UART_Port;
      Data    : out UART_Data_8b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
   is
      procedure Receive_Inner
        (This    : UART_Port;
         Data    : out UART_Data_8b;
         Status  : out UART_Status;
         Timeout : Natural := 1000;
         Periph : UART_Peripheral)
      with Relaxed_Initialization => Data,
        Post => (if Status = Ok then Data'Initialized);

      procedure Receive_Inner
        (This    : UART_Port;
         Data    : out UART_Data_8b;
         Status  : out UART_Status;
         Timeout : Natural := 1000;
         Periph : UART_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : UART_FIFO_Status;
         Enable_FIFOs : Boolean;
         Is_BE : Boolean;
         Is_FE : Boolean;
         Is_PE : Boolean;
      begin
         if Timeout > 0 then
            Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
         end if;

         for I in Data'Range loop
            pragma Loop_Invariant (for all J in Data'First .. I - 1 => Data(J)'Initialized);
            Enable_FIFOs := This.Config.Enable_FIFOs;
            if Enable_FIFOs then
               loop
                  FIFO := Receive_Status (This);
                  exit when FIFO = Not_Full or FIFO = Full;
                  if FIFO = Invalid then
                     Status := Err_Error;
                     return;
                  end if;

                  if Timeout > 0 and then RP.Timer.Clock >= Deadline then
                     Status := Err_Timeout;
                     return;
                  end if;
               end loop;
            end if;

            --  Read the whole UARTDR at once so that we get the flags
            --  synchronized with the DATA read.
            DR := Periph.UARTDR;
            Data (I) := DR.DATA;
            Is_BE := DR.BE;
            Is_FE := DR.FE;
            Is_PE := DR.PE;
            if Is_BE then
               Status := Busy;
               return;
            elsif Is_FE or Is_PE then
               Status := Err_Error;
               return;
            end if;
         end loop;
         Status := Ok;
      end Receive_Inner;
   begin
      case This.Num is
         when 0 => Receive_Inner (This, Data, Status, Timeout, UART0_Periph);
         when 1 => Receive_Inner (This, Data, Status, Timeout, UART1_Periph);
      end case;
   end Receive;

   --  overriding
   procedure Transmit
     (This    : in out UART_Port;
      Data    : UART_Data_9b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Data);
      pragma Unreferenced (This, Timeout);
   begin
      --  9-bit data is not supported by this hardware
      Status := Err_Error;
   end Transmit;

   --  overriding
   procedure Receive
     (This    : in out UART_Port;
      Data    : out UART_Data_9b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Data);
      pragma Unreferenced (This, Timeout);
   begin
      --  9-bit data is not supported by this hardware
      Status := Err_Error;
   end Receive;

   function Div_Integer
     (D : UART_Divider)
       return UARTIBRD_BAUD_DIVINT_Field
   is
      I : constant Natural := Natural (D);
      First : constant Natural := Natural (UART_Divider'First);
      Last : constant Natural := Natural (UART_Divider'Last);
   begin
      if I = Last or else UART_Divider_Base (I) > D then
         return UARTIBRD_BAUD_DIVINT_Field (I - 1);
      else
         return UARTIBRD_BAUD_DIVINT_Field (I);
      end if;
   end Div_Integer;

   function Div_Fraction
     (D : UART_Divider)
       return UARTFBRD_BAUD_DIVFRAC_Field
   --  with SPARK_Mode => Off
   is
      Multiple : constant UART_Divider := UART_Divider (2 ** UARTFBRD_BAUD_DIVFRAC_Field'Size);
      Int      : UART_Divider;
   begin
      if Div_Integer (D) > 0 then
         Int := UART_Divider (Div_Integer (D));
         return UARTFBRD_BAUD_DIVFRAC_Field ((D - Int) * Multiple);
      else
         return UARTFBRD_BAUD_DIVFRAC_Field (D * Multiple);
      end if;
   end Div_Fraction;

   function Div_Value
     (Int  : UARTIBRD_BAUD_DIVINT_Field;
      Frac : UARTFBRD_BAUD_DIVFRAC_Field)
       return UART_Divider
   is
   begin
      if Int > 0 then
         if Frac > 0 then
            return UART_Divider (Int) + (UART_Divider (Frac) / UART_Divider (2 ** UARTFBRD_BAUD_DIVFRAC_Field'Size));
         else
            return UART_Divider (Int);
         end if;
      else
         return UART_Divider (Frac) / UART_Divider (2 ** UARTFBRD_BAUD_DIVFRAC_Field'Size);
      end if;
   end Div_Value;

   procedure Set_FIFO_IRQ_Level (This : in out UART_Port;
                                 RX   :        FIFO_IRQ_Level;
                                 TX   :        FIFO_IRQ_Level)
   is
      procedure Set_FIFO_IRQ_Level_Inner (RX   :        FIFO_IRQ_Level;
                                          TX   :        FIFO_IRQ_Level;
                                          Periph : in out UART_Peripheral)
      is
      begin
         Periph.UARTIFLS := (TXIFLSEL => TX'Enum_Rep,
                             RXIFLSEL => RX'Enum_Rep,
                             others   => <>);
      end Set_FIFO_IRQ_Level_Inner;
   begin
      case This.Num is
         when 0 => Set_FIFO_IRQ_Level_Inner (RX, TX, UART0_Periph);
         when 1 => Set_FIFO_IRQ_Level_Inner (RX, TX, UART1_Periph);
      end case;
   end Set_FIFO_IRQ_Level;

   procedure Enable_IRQ (This : in out UART_Port;
                         IRQ  :        UART_IRQ_Flag)
   is
      procedure Enable_IRQ_Inner (IRQ  :        UART_IRQ_Flag;
                                  Periph : in out UART_Peripheral)
      is
      begin
         case IRQ is
         when Modem_RI => Periph.UARTIMSC.RIMIM := True;
         when Modem_CTS => Periph.UARTIMSC.CTSMIM := True;
         when Modem_DCD => Periph.UARTIMSC.DCDMIM := True;
         when Modem_DSR => Periph.UARTIMSC.DSRMIM := True;
         when Receive => Periph.UARTIMSC.RXIM := True;
         when Transmit => Periph.UARTIMSC.TXIM := True;
         when Receive_Timeout => Periph.UARTIMSC.RTIM := True;
         when Framing_Error => Periph.UARTIMSC.FEIM := True;
         when Parity_Error => Periph.UARTIMSC.PEIM := True;
         when Break_Error => Periph.UARTIMSC.BEIM := True;
         when Overrun_Error => Periph.UARTIMSC.OEIM := True;
         end case;
      end Enable_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Enable_IRQ_Inner (IRQ, UART0_Periph);
         when 1 => Enable_IRQ_Inner (IRQ, UART1_Periph);
      end case;
   end Enable_IRQ;

   procedure Disable_IRQ (This : in out UART_Port;
                          IRQ  :        UART_IRQ_Flag)
   is
      procedure Disable_IRQ_Inner (IRQ  :        UART_IRQ_Flag;
                                   Periph : in out UART_Peripheral)
      is
      begin
         case IRQ is
         when Modem_RI => Periph.UARTIMSC.RIMIM := False;
         when Modem_CTS => Periph.UARTIMSC.CTSMIM := False;
         when Modem_DCD => Periph.UARTIMSC.DCDMIM := False;
         when Modem_DSR => Periph.UARTIMSC.DSRMIM := False;
         when Receive => Periph.UARTIMSC.RXIM := False;
         when Transmit => Periph.UARTIMSC.TXIM := False;
         when Receive_Timeout => Periph.UARTIMSC.RTIM := False;
         when Framing_Error => Periph.UARTIMSC.FEIM := False;
         when Parity_Error => Periph.UARTIMSC.PEIM := False;
         when Break_Error => Periph.UARTIMSC.BEIM := False;
         when Overrun_Error => Periph.UARTIMSC.OEIM := False;
         end case;
      end Disable_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Disable_IRQ_Inner (IRQ, UART0_Periph);
         when 1 => Disable_IRQ_Inner (IRQ, UART1_Periph);
      end case;
   end Disable_IRQ;


   procedure Clear_IRQ (This : in out UART_Port;
                        IRQ  :        UART_IRQ_Flag)
   is
      procedure Clear_IRQ_Inner (IRQ  :        UART_IRQ_Flag;
                                 Periph : in out UART_Peripheral)
      is
      begin
         case IRQ is
         when Modem_RI => Periph.UARTICR := (RIMIC => True,
                                             Reserved_11_31 => <>,
                                             others => False);
         when Modem_CTS => Periph.UARTICR := (CTSMIC => True,
                                              Reserved_11_31 => <>,
                                              others => False);
         when Modem_DCD => Periph.UARTICR := (DCDMIC => True,
                                              Reserved_11_31 => <>,
                                              others => False);
         when Modem_DSR => Periph.UARTICR := (DSRMIC => True,
                                              Reserved_11_31 => <>,
                                              others => False);
         when Receive => Periph.UARTICR := (RXIC=> True,
                                            Reserved_11_31 => <>,
                                            others => False);
         when Transmit => Periph.UARTICR := (TXIC => True,
                                             Reserved_11_31 => <>,
                                             others => False);
         when Receive_Timeout => Periph.UARTICR := (RTIC => True,
                                                    Reserved_11_31 => <>,
                                                    others => False);
         when Framing_Error => Periph.UARTICR := (FEIC => True,
                                                  Reserved_11_31 => <>,
                                                  others => False);
         when Parity_Error => Periph.UARTICR := (PEIC => True,
                                                 Reserved_11_31 => <>,
                                                 others => False);
         when Break_Error => Periph.UARTICR := (BEIC => True,
                                                Reserved_11_31 => <>,
                                                others => False);
         when Overrun_Error => Periph.UARTICR := (OEIC => True,
                                                  Reserved_11_31 => <>,
                                                  others => False);
         end case;
      end Clear_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Clear_IRQ_Inner (IRQ, UART0_Periph);
         when 1 => Clear_IRQ_Inner (IRQ, UART1_Periph);
      end case;
   end Clear_IRQ;

   function Masked_IRQ_Status (This : UART_Port;
                               IRQ  : UART_IRQ_Flag)
                               return Boolean
   is
      function Masked_IRQ_Status_Inner (IRQ  : UART_IRQ_Flag;
                                        Periph : UART_Peripheral)
                                                 return Boolean
        with Volatile_Function;

      function Masked_IRQ_Status_Inner (IRQ  : UART_IRQ_Flag;
                                        Periph : UART_Peripheral)
                                           return Boolean
      is
      begin
         case IRQ is
         when Modem_RI => return Periph.UARTMIS.RIMMIS;
         when Modem_CTS => return Periph.UARTMIS.CTSMMIS;
         when Modem_DCD => return Periph.UARTMIS.DCDMMIS;
         when Modem_DSR => return Periph.UARTMIS.DSRMMIS;
         when Receive => return Periph.UARTMIS.RXMIS;
         when Transmit => return Periph.UARTMIS.TXMIS;
         when Receive_Timeout => return Periph.UARTMIS.RTMIS;
         when Framing_Error => return Periph.UARTMIS.FEMIS ;
         when Parity_Error => return Periph.UARTMIS.PEMIS;
         when Break_Error => return Periph.UARTMIS.BEMIS;
         when Overrun_Error => return Periph.UARTMIS.OEMIS;
         end case;
      end Masked_IRQ_Status_Inner;
   begin
      case This.Num is
         when 0 => return Masked_IRQ_Status_Inner (IRQ, UART0_Periph);
         when 1 => return Masked_IRQ_Status_Inner (IRQ, UART1_Periph);
      end case;
   end Masked_IRQ_Status;

   function Raw_IRQ_Status (This : UART_Port;
                            IRQ  : UART_IRQ_Flag)
                            return Boolean
   is
      function Raw_IRQ_Status_Inner (IRQ  : UART_IRQ_Flag;
                                     Periph : UART_Peripheral)
                                     return Boolean
        with Volatile_Function;

      function Raw_IRQ_Status_Inner (IRQ  : UART_IRQ_Flag;
                                     Periph : UART_Peripheral)
                                        return Boolean
      is
      begin
         case IRQ is
         when Modem_RI => return Periph.UARTRIS.RIRMIS;
         when Modem_CTS => return Periph.UARTRIS.CTSRMIS;
         when Modem_DCD => return Periph.UARTRIS.DCDRMIS;
         when Modem_DSR => return Periph.UARTRIS.DSRRMIS;
         when Receive => return Periph.UARTRIS.RXRIS;
         when Transmit => return Periph.UARTRIS.TXRIS;
         when Receive_Timeout => return Periph.UARTRIS.RTRIS;
         when Framing_Error => return Periph.UARTRIS.FERIS;
         when Parity_Error => return Periph.UARTRIS.PERIS;
         when Break_Error => return Periph.UARTRIS.BERIS;
         when Overrun_Error => return Periph.UARTRIS.OERIS;
         end case;
      end Raw_IRQ_Status_Inner;
   begin
      case This.Num is
         when 0 => return Raw_IRQ_Status_Inner (IRQ, UART0_Periph);
         when 1 => return Raw_IRQ_Status_Inner (IRQ, UART1_Periph);
      end case;
   end Raw_IRQ_Status;

end RP.UART;
