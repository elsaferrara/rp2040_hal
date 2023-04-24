--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP2040_SVD.SPI; use RP2040_SVD.SPI;
with RP.Timer;
with RP.Reset;
with HAL; use HAL;

package body RP.SPI with SPARK_Mode is
   procedure Configure
      (This   : in out SPI_Port;
       Config : SPI_Configuration := Default_SPI_Configuration)
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

      This.Periph.SSPCR0 :=
         (FRF    => 0,       --  Motorola format
          SCR    => 0,       --  No divider before Set_Speed is called
          SPO    => Config.Polarity = Active_High,
          SPH    => Config.Phase = Falling_Edge,
          others => <>);

      case Config.Data_Size is
         when Data_Size_8b =>
            This.Periph.SSPCR0.DSS := 2#0111#;
         when Data_Size_16b =>
            This.Periph.SSPCR0.DSS := 2#1111#;
      end case;

      This.Periph.SSPCR1 :=
         (MS     => Config.Role = Slave,
          SSE    => False,
          LBM    => Config.Loopback,
          others => <>);

      --  Enable DMA request. Harmless if DMA is not used.
      This.Periph.SSPDMACR :=
         (RXDMAE => True,
          TXDMAE => True,
          others => <>);

      This.Set_Speed (Config.Baud);

      This.Blocking := Config.Blocking;

      This.Periph.SSPCR1.SSE := True;
   end Configure;

   procedure Set_Speed
      (This : in out SPI_Port;
       Baud : Hertz)
   is
      Baud64   : constant UInt64 := UInt64 (Baud);
      Clock_Frequency : Hertz;
      Freq_In  : UInt64;
      Prescale : UInt64 := 2;
      Postdiv  : UInt64 := 256;
   begin
      RP.Clock.Frequency (RP.Clock.PERI, Clock_Frequency);
      Freq_In := UInt64 (Clock_Frequency);
      while Prescale <= 254 loop
         exit when Freq_In < (Prescale + 2) * 256 * Baud64;
         Prescale := Prescale + 2;
      end loop;
      if Prescale > 254 then
         raise Clock_Speed_Error with "PERI frequency too low for requested SPI baud";
      end if;

      while Postdiv > 1 loop
         exit when Freq_In / (Prescale * (Postdiv - 1)) > Baud64;
         Postdiv := Postdiv - 1;
      end loop;

      This.Periph.SSPCPSR.CPSDVSR := SSPCPSR_CPSDVSR_Field (Prescale);
      This.Periph.SSPCR0.SCR := SSPCR0_SCR_Field (Postdiv - 1);
   end Set_Speed;

   function Transmit_Status
      (This : SPI_Port)
      return SPI_FIFO_Status
   is
      --  This is a bit dumb, but we want to avoid redefining the whole
      --  SPI_Peripheral record just to change the status registers.
      --
      --  TFE   TNF   Returns     Notes
      --   0     0    Full
      --   0     1    Not_Full    some data in FIFO
      --   1     0    Invalid     cannot be both Empty and Full
      --   1     1    Empty

      Is_TFE : constant Boolean := This.Periph.SSPSR.TFE;
      Is_TNF : constant Boolean := This.Periph.SSPSR.TNF;
      Is_BSY : constant Boolean := This.Periph.SSPSR.BSY;
   begin
      if not Is_TFE and not Is_TNF then
         return Full;
      elsif not Is_TFE and Is_TNF then
         return Not_Full;
      elsif Is_BSY then
         return Busy;
      elsif Is_TFE and Is_TNF then
         return Empty;
      else
         return Invalid;
      end if;
   end Transmit_Status;

   function Receive_Status
      (This : SPI_Port)
      return SPI_FIFO_Status
   is
      --  RFF  RNE   Returns     Notes
      --   0    0    Empty
      --   0    1    Not_Full
      --   1    0    Invalid     cannot be both Empty and Full
      --   1    1    Full

      Is_RFF : constant Boolean := This.Periph.SSPSR.RFF;
      Is_RNE : constant Boolean := This.Periph.SSPSR.RNE;
      Is_BSY : constant Boolean := This.Periph.SSPSR.BSY;
   begin
      if not Is_RFF and not Is_RNE then
         return Empty;
      elsif Is_BSY then
         return Busy;
      elsif not Is_RFF and Is_RNE then
         return Not_Full;
      elsif Is_RFF and Is_RNE then
         return Full;
      else
         return Invalid;
      end if;
   end Receive_Status;

   function FIFO_Address
      (This : SPI_Port)
      return System.Address
   is (This.Periph.SSPDR'Address);

   --  overriding
   function Data_Size
      (This : SPI_Port)
      return SPI_Data_Size
   is
   begin
      if This.Periph.SSPCR0.DSS = 2#1111# then
         return Data_Size_16b;
      else
         return Data_Size_8b;
      end if;
   end Data_Size;

   --  overriding
   procedure Transmit
      (This    : in out SPI_Port;
       Data    : SPI_Data_8b;
       Status  : out SPI_Status;
       Timeout : Natural := 1000)
   is
      use type RP.Timer.Time;
      Deadline : RP.Timer.Time;
      FIFO     : SPI_FIFO_Status;
   begin
      if Timeout > 0 then
         Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
      end if;

      for D of Data loop
         loop
            FIFO := Transmit_Status (This);
            exit when FIFO = Empty or FIFO = Not_Full;

            if FIFO = Invalid then
               Status := Err_Error;
               return;
            end if;

            if Timeout > 0 and then RP.Timer.Clock >= Deadline then
               Status := Err_Timeout;
               return;
            end if;
         end loop;
         This.Periph.SSPDR.DATA := SSPDR_DATA_Field (D);
      end loop;

      if This.Blocking then
         while This.Transmit_Status /= Empty loop
            if Timeout > 0 and then RP.Timer.Clock >= Deadline then
               Status := Err_Timeout;
               return;
            end if;
         end loop;
      end if;

      Status := Ok;
   end Transmit;

   --  overriding
   procedure Transmit
      (This    : in out SPI_Port;
       Data    : SPI_Data_16b;
       Status  : out SPI_Status;
       Timeout : Natural := 1000)
   is
      use type RP.Timer.Time;
      Deadline : RP.Timer.Time;
      FIFO     : SPI_FIFO_Status;
   begin
      if Timeout > 0 then
         Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
      end if;

      for D of Data loop
         loop
            FIFO := Transmit_Status (This);
            exit when FIFO = Empty or FIFO = Not_Full;

            if FIFO = Invalid then
               Status := Err_Error;
               return;
            end if;

            if Timeout > 0 and then RP.Timer.Clock >= Deadline then
               Status := Err_Timeout;
               return;
            end if;
         end loop;
         This.Periph.SSPDR.DATA := D;
      end loop;

      if This.Blocking then
         while This.Transmit_Status /= Empty loop
            if Timeout > 0 and then RP.Timer.Clock >= Deadline then
               Status := Err_Timeout;
               return;
            end if;
         end loop;
      end if;

      Status := Ok;
   end Transmit;

   --  overriding
   procedure Receive
      (This    : in out SPI_Port;
       Data    : out SPI_Data_8b;
       Status  : out SPI_Status;
       Timeout : Natural := 1000)
   is
      use type RP.Timer.Time;
      Deadline : RP.Timer.Time;
      FIFO     : SPI_FIFO_Status;
   begin
      if Timeout > 0 then
         Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
      end if;

      for I in Data'Range loop
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
         Data (I) := UInt8 (This.Periph.SSPDR.DATA);
      end loop;
      Status := Ok;
   end Receive;

   --  overriding
   procedure Receive
      (This    : in out SPI_Port;
       Data    : out SPI_Data_16b;
       Status  : out SPI_Status;
       Timeout : Natural := 1000)
   is
      use type RP.Timer.Time;
      Deadline : RP.Timer.Time;
      FIFO     : SPI_FIFO_Status;
   begin
      if Timeout > 0 then
         Deadline := RP.Timer.Clock + RP.Timer.Milliseconds (Timeout);
      end if;

      for I in Data'Range loop
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
         Data (I) := UInt16 (This.Periph.SSPDR.DATA);
      end loop;
      Status := Ok;
   end Receive;

   procedure Enable_IRQ (This : in out SPI_Port;
                         IRQ  :        SPI_IRQ_Flag)
   is
   begin
      case IRQ is
         when Receive_Overrun => This.Periph.SSPIMSC.RORIM := True;
         when Receive_FIFO_Not_Empty => This.Periph.SSPIMSC.RTIM := True;
         when Receive_FIFO_Half_Full => This.Periph.SSPIMSC.RXIM := True;
         when Transmit_FIFO_Half_Empty => This.Periph.SSPIMSC.TXIM := True;
      end case;
   end Enable_IRQ;

   procedure Disable_IRQ (This : in out SPI_Port;
                          IRQ  :        SPI_IRQ_Flag)
   is
   begin
      case IRQ is
         when Receive_Overrun => This.Periph.SSPIMSC.RORIM := False;
         when Receive_FIFO_Not_Empty => This.Periph.SSPIMSC.RTIM := False;
         when Receive_FIFO_Half_Full => This.Periph.SSPIMSC.RXIM := False;
         when Transmit_FIFO_Half_Empty => This.Periph.SSPIMSC.TXIM := False;
      end case;
   end Disable_IRQ;

   procedure Clear_IRQ (This : in out SPI_Port;
                        IRQ  :        SPI_IRQ_Flag)
   is
   begin
      case IRQ is
         when Receive_Overrun => This.Periph.SSPICR.RORIC := True;
         when Receive_FIFO_Not_Empty => This.Periph.SSPICR.RTIC := True;
         when Receive_FIFO_Half_Full => null;
         when Transmit_FIFO_Half_Empty => null;
      end case;
   end Clear_IRQ;

   function Masked_IRQ_Status (This : SPI_Port;
                               IRQ  : SPI_IRQ_Flag)
                               return Boolean
   is
   begin
      case IRQ is
         when Receive_Overrun => return This.Periph.SSPMIS.RORMIS;
         when Receive_FIFO_Not_Empty => return This.Periph.SSPMIS.RTMIS;
         when Receive_FIFO_Half_Full => return This.Periph.SSPMIS.RXMIS;
         when Transmit_FIFO_Half_Empty => return This.Periph.SSPMIS.TXMIS;
      end case;
   end Masked_IRQ_Status;

   function Raw_IRQ_Status (This : SPI_Port;
                            IRQ  : SPI_IRQ_Flag)
                            return Boolean
   is
   begin
      case IRQ is
         when Receive_Overrun => return This.Periph.SSPRIS.RORRIS;
         when Receive_FIFO_Not_Empty => return This.Periph.SSPRIS.RTRIS;
         when Receive_FIFO_Half_Full => return This.Periph.SSPRIS.RXRIS;
         when Transmit_FIFO_Half_Empty => return This.Periph.SSPRIS.TXRIS;
      end case;
   end Raw_IRQ_Status;

end RP.SPI;
