--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP.Timer;
with RP.Reset;
with HAL; use HAL;

package body RP.SPI with SPARK_Mode is

   procedure Set_Speed_Inner
     (Baud : Hertz;
      Periph : in out SPI_Peripheral);

   procedure Configure
     (This   : out SPI_Port;
      Config : SPI_Configuration := Default_SPI_Configuration)
   is
      procedure Configure_Inner
        (This   : out SPI_Port;
         Config : SPI_Configuration := Default_SPI_Configuration;
         Periph : in out SPI_Peripheral)
      is
         use RP.Reset;
      begin
         RP.Clock.Enable (RP.Clock.PERI);
         case This.Num is
         when 0 => Reset_Peripheral (Reset_SPI0);
         when 1 => Reset_Peripheral (Reset_SPI1);
         end case;

         --  clk_peri   := clk_sys
         --  fSSPCLK    := clk_peri
         --  fSSPCLKOUT := fSSPCLK / (CPSDVSR * (1 + SCR);

         Periph.SSPCR0 :=
           (FRF    => 0,       --  Motorola format
            SCR    => 0,       --  No divider before Set_Speed is called
            SPO    => Config.Polarity = Active_High,
            SPH    => Config.Phase = Falling_Edge,
            others => <>);

         case Config.Data_Size is
         when Data_Size_8b =>
            Periph.SSPCR0.DSS := 2#0111#;
         when Data_Size_16b =>
            Periph.SSPCR0.DSS := 2#1111#;
         end case;

         Periph.SSPCR1 :=
           (MS     => Config.Role = Slave,
            SSE    => False,
            LBM    => Config.Loopback,
            others => <>);

         --  Enable DMA request. Harmless if DMA is not used.
         Periph.SSPDMACR :=
           (RXDMAE => True,
            TXDMAE => True,
            others => <>);

          Set_Speed_Inner (Config.Baud, Periph);

         This.Blocking := Config.Blocking;

         Periph.SSPCR1.SSE := True;
      end Configure_Inner;
   begin
      case This.Num is
         when 0 => Configure_Inner (This, Config, SPI0_Periph);
         when 1 => Configure_Inner (This, Config, SPI1_Periph);
      end case;
   end Configure;

   procedure Set_Speed_Inner
        (Baud : Hertz;
         Periph : in out SPI_Peripheral)
      is
         Baud64   : constant UInt64 := UInt64 (Baud);
         Clock_Frequency : Hertz;
         Freq_In  : UInt64;
         Prescale : UInt64 := 2;
         subtype Postdiv_type is UInt64 range 0 .. 256;
         Postdiv  : Postdiv_type := 256;
      begin
         RP.Clock.Frequency (RP.Clock.PERI, Clock_Frequency);
         Freq_In := UInt64 (Clock_Frequency);
         while Prescale <= 254 loop
            --  pragma Loop_Variant (Increases => Prescale);
            pragma Loop_Invariant (Prescale >= 2);
            exit when Freq_In < (Prescale + 2) * 256 * Baud64;
            Prescale := Prescale + 2;
         end loop;
         if Prescale > 254 then
            raise Clock_Speed_Error with "PERI frequency too low for requested SPI baud";
            pragma Annotate (GNATprove, Intentional, "exception might be raised", "Ignoring exception for now");
         end if;
         pragma Assert (Prescale >= 2);

         while Postdiv > 1 loop
            pragma Loop_Invariant (Prescale > 0);
            pragma Loop_Invariant (Postdiv > 1);
            pragma Loop_Invariant (Postdiv - 1 > 0);
            exit when Freq_In / Prescale / (Postdiv - 1) > Baud64;
            Postdiv := Postdiv - 1;
         end loop;

         Periph.SSPCPSR.CPSDVSR := SSPCPSR_CPSDVSR_Field (Prescale);
         Periph.SSPCR0.SCR := SSPCR0_SCR_Field (Postdiv - 1);
      end Set_Speed_Inner;

   procedure Set_Speed
     (This : SPI_Port;
      Baud : Hertz)
   is
   begin
      case This.Num is
         when 0 => Set_Speed_Inner (Baud, SPI0_Periph);
         when 1 => Set_Speed_Inner (Baud, SPI1_Periph);
      end case;
   end Set_Speed;

   procedure Transmit_Status_Inner
     (Periph : SPI_Peripheral;
      Result : out SPI_FIFO_Status)
   is
      --  This is a bit dumb, but we want to avoid redefining the whole
      --  SPI_Peripheral record just to change the status registers.
      --
      --  TFE   TNF   Returns     Notes
      --   0     0    Full
      --   0     1    Not_Full    some data in FIFO
      --   1     0    Invalid     cannot be both Empty and Full
      --   1     1    Empty

      Is_TFE : constant Boolean := Periph.SSPSR.TFE;
      Is_TNF : constant Boolean := Periph.SSPSR.TNF;
      Is_BSY : constant Boolean := Periph.SSPSR.BSY;
   begin
      if not Is_TFE and not Is_TNF then
         Result := Full;
      elsif not Is_TFE and Is_TNF then
         Result := Not_Full;
      elsif Is_BSY then
         Result := Busy;
      elsif Is_TFE and Is_TNF then
         Result := Empty;
      else
         Result := Invalid;
      end if;
   end Transmit_Status_Inner;

   procedure Transmit_Status
     (This : SPI_Port;
      Result : out SPI_FIFO_Status)
   is
   begin
      case This.Num is
         when 0 => Transmit_Status_Inner (SPI0_Periph, Result);
         when 1 => Transmit_Status_Inner (SPI1_Periph, Result);
      end case;
   end Transmit_Status;

   procedure Receive_Status_Inner
     (Periph : SPI_Peripheral;
      Result : out SPI_FIFO_Status)
   is
      --  RFF  RNE   Returns     Notes
      --   0    0    Empty
      --   0    1    Not_Full
      --   1    0    Invalid     cannot be both Empty and Full
      --   1    1    Full

      Is_RFF : constant Boolean := Periph.SSPSR.RFF;
      Is_RNE : constant Boolean := Periph.SSPSR.RNE;
      Is_BSY : constant Boolean := Periph.SSPSR.BSY;
   begin
      if not Is_RFF and not Is_RNE then
         Result := Empty;
      elsif Is_BSY then
         Result := Busy;
      elsif not Is_RFF and Is_RNE then
         Result := Not_Full;
      elsif Is_RFF and Is_RNE then
         Result := Full;
      else
         Result := Invalid;
      end if;
   end Receive_Status_Inner;

   procedure Receive_Status
     (This : SPI_Port;
      Result : out SPI_FIFO_Status)
   is
   begin
      case This.Num is
         when 0 => Receive_Status_Inner (SPI0_Periph, Result);
         when 1 => Receive_Status_Inner (SPI1_Periph, Result);
      end case;
   end Receive_Status;

   function FIFO_Address
     (This : SPI_Port)
      return System.Address
     with SPARK_Mode => Off
   is
      function FIFO_Address_Inner
        (Periph : SPI_Peripheral)
         return System.Address
        with Volatile_Function;

      function FIFO_Address_Inner
        (Periph : SPI_Peripheral)
         return System.Address
      is
      begin
         return Periph.SSPDR'Address;
      end FIFO_Address_Inner;
   begin
      case This.Num is
         when 0 => return FIFO_Address_Inner (SPI0_Periph);
         when 1 => return FIFO_Address_Inner (SPI1_Periph);
      end case;
   end FIFO_Address;

   --  overriding
   procedure Data_Size
     (This : SPI_Port;
      Result : out SPI_Data_Size)
   is
      procedure Data_Size_Inner
        (Periph : SPI_Peripheral;
         Result : out SPI_Data_Size)
      is
         DSS : constant SSPCR0_DSS_Field := Periph.SSPCR0.DSS;
      begin
         if DSS = 2#1111# then
            Result := Data_Size_16b;
         else
            Result := Data_Size_8b;
         end if;
      end Data_Size_Inner;
   begin
      case This.Num is
         when 0 => Data_Size_Inner (SPI0_Periph, Result);
         when 1 => Data_Size_Inner (SPI1_Periph, Result);
      end case;
   end Data_Size;

   --  overriding
   procedure Transmit
     (This    : SPI_Port;
      Data    : SPI_Data_8b;
      Status  : out SPI_Status;
      Timeout : Natural := 1000)
   is
      procedure Transmit_Inner
        (This    : SPI_Port;
         Data    : SPI_Data_8b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph : in out SPI_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : SPI_FIFO_Status;
         Current_Time : RP.Timer.Time;
      begin
         if Timeout > 0 then
            RP.Timer.Clock (Current_Time);
            Deadline := Current_Time + RP.Timer.Milliseconds (Timeout);
         end if;

         for D of Data loop
            loop
               Transmit_Status_Inner (Periph, FIFO);
               exit when FIFO = Empty or FIFO = Not_Full;

               if FIFO = Invalid then
                  Status := Err_Error;
                  return;
               end if;
               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
            Periph.SSPDR.DATA := SSPDR_DATA_Field (D);
         end loop;

         if This.Blocking then
            loop
               Transmit_Status_Inner (Periph, FIFO);
               exit when FIFO = Empty;
               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
         end if;

         Status := Ok;
      end Transmit_Inner;
   begin
      case This.Num is
         when 0 => Transmit_Inner (This, Data, Status, Timeout, SPI0_Periph);
         when 1 => Transmit_Inner (This, Data, Status, Timeout, SPI1_Periph);
      end case;
   end Transmit;

   --  overriding
   procedure Transmit
     (This    : SPI_Port;
      Data    : SPI_Data_16b;
      Status  : out SPI_Status;
      Timeout : Natural := 1000)
   is
      procedure Transmit_Inner
        (This    : SPI_Port;
         Data    : SPI_Data_16b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph : in out SPI_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : SPI_FIFO_Status;
         Current_Time : RP.Timer.Time;
      begin
         if Timeout > 0 then
            RP.Timer.Clock (Current_Time);
            Deadline := Current_Time + RP.Timer.Milliseconds (Timeout);
         end if;

         for D of Data loop
            loop
               Transmit_Status_Inner (Periph, FIFO);
               exit when FIFO = Empty or FIFO = Not_Full;

               if FIFO = Invalid then
                  Status := Err_Error;
                  return;
               end if;

               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
            Periph.SSPDR.DATA := D;
         end loop;

         if This.Blocking then
            loop
               Transmit_Status_Inner (Periph, FIFO);
               exit when FIFO = Empty;
               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
         end if;

         Status := Ok;
      end Transmit_Inner;
   begin
      case This.Num is
         when 0 => Transmit_Inner (This, Data, Status, Timeout, SPI0_Periph);
         when 1 => Transmit_Inner (This, Data, Status, Timeout, SPI1_Periph);
      end case;
   end Transmit;

   --  overriding
   procedure Receive
     (This    : SPI_Port;
      Data    : out SPI_Data_8b;
      Status  : out SPI_Status;
      Timeout : Natural := 1000)
   is
      procedure Receive_Inner
        (Data    : out SPI_Data_8b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph  : SPI_Peripheral)
        with Relaxed_Initialization => Data,
        Post => (if Status = Ok then Data'Initialized);

      procedure Receive_Inner
        (Data    : out SPI_Data_8b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph  : SPI_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : SPI_FIFO_Status;
         Periph_Data : UInt16;
         Current_Time : RP.Timer.Time;
      begin
         if Timeout > 0 then
            RP.Timer.Clock (Current_Time);
            Deadline := Current_Time + RP.Timer.Milliseconds (Timeout);
         end if;

         for I in Data'Range loop
            pragma Loop_Invariant (for all J in Data'First .. I - 1 => Data (J)'Initialized);
            loop
               Receive_Status_Inner (Periph, FIFO);
               exit when FIFO = Not_Full or FIFO = Full;

               if FIFO = Invalid then
                  Status := Err_Error;
                  return;
               end if;

               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
            Periph_Data := Periph.SSPDR.DATA;
            Data (I) := UInt8 (Periph_Data and 16#00ff#);
         end loop;
         Status := Ok;
      end Receive_Inner;
   begin
      case This.Num is
         when 0 => Receive_Inner (Data, Status, Timeout, SPI0_Periph);
         when 1 => Receive_Inner (Data, Status, Timeout, SPI1_Periph);
      end case;
   end Receive;

   --  overriding
   procedure Receive
     (This    : SPI_Port;
      Data    : out SPI_Data_16b;
      Status  : out SPI_Status;
      Timeout : Natural := 1000)
   is

      procedure Receive_Inner
        (Data    : out SPI_Data_16b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph  : SPI_Peripheral)
        with Relaxed_Initialization => Data,
        Post => (if Status = Ok then Data'Initialized);

      procedure Receive_Inner
        (Data    : out SPI_Data_16b;
         Status  : out SPI_Status;
         Timeout : Natural := 1000;
         Periph  : SPI_Peripheral)
      is
         use type RP.Timer.Time;
         Deadline : RP.Timer.Time with Relaxed_Initialization;
         FIFO     : SPI_FIFO_Status;
         Current_Time : RP.Timer.Time;
      begin
         if Timeout > 0 then
            RP.Timer.Clock (Current_Time);
            Deadline := Current_Time + RP.Timer.Milliseconds (Timeout);
         end if;

         for I in Data'Range loop
            pragma Loop_Invariant (for all J in Data'First .. I - 1 => Data (J)'Initialized);
            loop
               Receive_Status_Inner (Periph, FIFO);
               exit when FIFO = Not_Full or FIFO = Full;

               if FIFO = Invalid then
                  Status := Err_Error;
                  return;
               end if;

               RP.Timer.Clock (Current_Time);
               if Timeout > 0 and then Current_Time >= Deadline then
                  Status := Err_Timeout;
                  return;
               end if;
            end loop;
            Data (I) := UInt16 (Periph.SSPDR.DATA);
         end loop;
         Status := Ok;
      end Receive_Inner;
   begin
      case This.Num is
         when 0 => Receive_Inner (Data, Status, Timeout, SPI0_Periph);
         when 1 => Receive_Inner (Data, Status, Timeout, SPI1_Periph);
      end case;
   end Receive;

   procedure Enable_IRQ (This :        SPI_Port;
                         IRQ  :        SPI_IRQ_Flag)
   is
      procedure Enable_IRQ_Inner (IRQ  :        SPI_IRQ_Flag;
                                  Periph : in out SPI_Peripheral)
      is
      begin
         case IRQ is
         when Receive_Overrun => Periph.SSPIMSC.RORIM := True;
         when Receive_FIFO_Not_Empty => Periph.SSPIMSC.RTIM := True;
         when Receive_FIFO_Half_Full => Periph.SSPIMSC.RXIM := True;
         when Transmit_FIFO_Half_Empty => Periph.SSPIMSC.TXIM := True;
         end case;
      end Enable_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Enable_IRQ_Inner (IRQ, SPI0_Periph);
         when 1 => Enable_IRQ_Inner (IRQ, SPI1_Periph);
      end case;
   end Enable_IRQ;

   procedure Disable_IRQ (This :        SPI_Port;
                          IRQ  :        SPI_IRQ_Flag)
   is
      procedure Disable_IRQ_Inner (IRQ  :        SPI_IRQ_Flag;
                                   Periph : in out SPI_Peripheral)
      is
      begin
         case IRQ is
         when Receive_Overrun => Periph.SSPIMSC.RORIM := False;
         when Receive_FIFO_Not_Empty => Periph.SSPIMSC.RTIM := False;
         when Receive_FIFO_Half_Full => Periph.SSPIMSC.RXIM := False;
         when Transmit_FIFO_Half_Empty => Periph.SSPIMSC.TXIM := False;
         end case;
      end Disable_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Disable_IRQ_Inner (IRQ, SPI0_Periph);
         when 1 => Disable_IRQ_Inner (IRQ, SPI1_Periph);
      end case;
   end Disable_IRQ;

   procedure Clear_IRQ (This :        SPI_Port;
                        IRQ  :        SPI_IRQ_Flag)
   is
      procedure Clear_IRQ_Inner (IRQ  :        SPI_IRQ_Flag;
                                 Periph : in out SPI_Peripheral)
      is
      begin
         case IRQ is
         when Receive_Overrun => Periph.SSPICR.RORIC := True;
         when Receive_FIFO_Not_Empty => Periph.SSPICR.RTIC := True;
         when Receive_FIFO_Half_Full => null;
         when Transmit_FIFO_Half_Empty => null;
         end case;
      end Clear_IRQ_Inner;
   begin
      case This.Num is
         when 0 => Clear_IRQ_Inner (IRQ, SPI0_Periph);
         when 1 => Clear_IRQ_Inner (IRQ, SPI1_Periph);
      end case;
   end Clear_IRQ;

   procedure Masked_IRQ_Status (This : SPI_Port;
                                IRQ  : SPI_IRQ_Flag;
                                Result : out Boolean)
   is
      procedure Masked_IRQ_Status_Inner (IRQ  : SPI_IRQ_Flag;
                                         Periph : SPI_Peripheral;
                                         Result : out Boolean)
      is
      begin
         case IRQ is
         when Receive_Overrun => Result := Periph.SSPMIS.RORMIS;
         when Receive_FIFO_Not_Empty => Result := Periph.SSPMIS.RTMIS;
         when Receive_FIFO_Half_Full => Result := Periph.SSPMIS.RXMIS;
         when Transmit_FIFO_Half_Empty => Result := Periph.SSPMIS.TXMIS;
         end case;
      end Masked_IRQ_Status_Inner;
   begin
      case This.Num is
         when 0 => Masked_IRQ_Status_Inner (IRQ, SPI0_Periph, Result);
         when 1 => Masked_IRQ_Status_Inner (IRQ, SPI1_Periph, Result);
      end case;
   end Masked_IRQ_Status;

   procedure Raw_IRQ_Status (This : SPI_Port;
                             IRQ  : SPI_IRQ_Flag;
                             Result : out Boolean)
   is
      procedure Raw_IRQ_Status_Inner (IRQ  : SPI_IRQ_Flag;
                                      Periph : SPI_Peripheral;
                                      Result : out Boolean)
      is
      begin
         case IRQ is
         when Receive_Overrun => Result := Periph.SSPRIS.RORRIS;
         when Receive_FIFO_Not_Empty => Result := Periph.SSPRIS.RTRIS;
         when Receive_FIFO_Half_Full => Result := Periph.SSPRIS.RXRIS;
         when Transmit_FIFO_Half_Empty => Result := Periph.SSPRIS.TXRIS;
         end case;
      end Raw_IRQ_Status_Inner;
   begin
      case This.Num is
         when 0 => Raw_IRQ_Status_Inner (IRQ, SPI0_Periph, Result);
         when 1 => Raw_IRQ_Status_Inner (IRQ, SPI1_Periph, Result);
      end case;
   end Raw_IRQ_Status;

end RP.SPI;
