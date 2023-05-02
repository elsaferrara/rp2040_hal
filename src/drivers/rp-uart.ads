--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP2040_SVD.UART; use RP2040_SVD.UART;
with HAL.UART; use HAL.UART;
with HAL.Time;
with RP.Clock;
with System;
with HAL; use HAL;

package RP.UART
   with Preelaborate, Elaborate_Body, SPARK_Mode
is
   subtype UART_Word_Size is Integer range 5 .. 8;
   subtype UART_Stop_Bits is Integer range 1 .. 2;
   subtype UART_Frame_Length is Positive range 1 .. 1;

   type UART_Parity_Type is (Even, Odd);

   type UART_FIFO_Status is (Empty, Not_Full, Full, Busy, Invalid);

   subtype Hertz_Baud is Hertz range 2 ** 11 .. Natural'Last;

   --  Default configuration is 115200 8n1
   --  https://en.wikipedia.org/wiki/8-N-1
   type UART_Configuration is record
      Baud         : Hertz_Baud := 115_200;
      Word_Size    : UART_Word_Size := 8;
      Parity       : Boolean := False;
      Stop_Bits    : UART_Stop_Bits := 1;
      Parity_Type  : UART_Parity_Type := Even; --  has no effect when Parity = False
      Frame_Length : UART_Frame_Length := 1; --  Words per frame. Used to calculate break timing.
      Loopback     : Boolean := False;
      Enable_FIFOs : Boolean := True; -- Enable TX and RX FIFOs
   end record;

   Default_UART_Configuration : constant UART_Configuration := (others => <>);

   subtype UART_Number is Natural range 0 .. 1;


   type UART_Port
     (Num    : UART_Number)
   is tagged -- new HAL.UART.UART_Port with
      record
          --  Periph : RP2040_SVD.UART.UART_Peripheral;
         Config : UART_Configuration;
      end record;

   --  type UART_Port
   --     (Num    : UART_Number;
   --      Periph : not null access RP2040_SVD.UART.UART_Peripheral)
   --  is tagged record
   --     Config : UART_Configuration;
   --  end record;

   procedure Configure
      (This   : out UART_Port;
       Config : UART_Configuration := Default_UART_Configuration)
     with Pre'Class => Config.Baud < 2 ** 27
   ;

   --  If parity is enabled, the parity bit may be forced high using this
   --  procedure. Stick parity is used in some protocols to indicate the
   --  beginning of a new message.
   procedure Set_Stick_Parity
      (This    : in UART_Port;
       Enabled : Boolean);

   --  Just so we're clear on the magnitude of these timings
   subtype Microseconds is Integer;

   --  Duration of a single mark or space symbol for the current configuration
   function Symbol_Time
      (This : UART_Port)
       return Microseconds
     with Post => Symbol_Time'Result <= 1_000_001
   and then Symbol_Time'Result > 0;


   --  Duration of a single frame transmission for the current configuration
   function Frame_Time
      (This : UART_Port)
       return Microseconds
     with Post => Frame_Time'Result >= 0
       and then Frame_Time'Result < 2 ** 30
   ;

   --  Send a break by holding TX active. The Delays implementation must
   --  support Delay_Microseconds. It's okay if delays are longer, but they
   --  cannot be shorter. If Start = True, an additional delay of one bit
   --  period will be added before the break.
   generic
   type T (<>) is new HAL.Time.Delays with private;
   procedure Send_Break
     (This     : UART_Port;
      Delays   : in out T;
      Duration : Microseconds;
      Start    : Boolean := True);

   procedure Transmit_Status_Inner
     (Periph : UART_Peripheral;
      Result : out UART_FIFO_Status);

   procedure Transmit_Status
     (This : UART_Port;
      Result : out UART_FIFO_Status);

   procedure Receive_Status_Inner
     (Periph : UART_Peripheral;
      Result : out UART_FIFO_Status);

   procedure Receive_Status
     (This : UART_Port;
      Result : out UART_FIFO_Status);

   function FIFO_Address
      (This : UART_Port)
      return System.Address
   with Volatile_Function;

   --  overriding
   function Data_Size
      (Port : UART_Port)
      return UART_Data_Size;

   --  overriding
   procedure Transmit
     (This    : UART_Port;
      Data    : UART_Data_8b;
      Status  : out UART_Status;
      Timeout : Natural := 1000);

   --  overriding
   procedure Transmit
     (This    : in out UART_Port;
      Data    : UART_Data_9b;
      Status  : out UART_Status;
      Timeout : Natural := 1000);

   --  overriding
   procedure Receive
     (This    : UART_Port;
      Data    : out UART_Data_8b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
     with Relaxed_Initialization => Data,
   Post => (if Status = Ok then Data'Initialized);

   --  overriding
   procedure Receive
     (This    : in out UART_Port;
      Data    : out UART_Data_9b;
      Status  : out UART_Status;
      Timeout : Natural := 1000)
   with Relaxed_Initialization => Data;

   type FIFO_IRQ_Level is (Lvl_Eighth,
                           Lvl_Quarter,
                           Lvl_Half,
                           Lvl_Three_Quarter,
                           Lvl_Seven_Eighth);

   procedure Set_FIFO_IRQ_Level (This :        UART_Port;
                                 RX   :        FIFO_IRQ_Level;
                                 TX   :        FIFO_IRQ_Level);
   --  Set the trigger point for receive and transmit FIFO interrupt. For the
   --  receive FIFO, the interrupt is triggered when the FIFO level is above or
   --  equal to the set level. For the transmit FIFO, the interrupt is triggered
   --  when the FIFO level is below or equal to the set level.

   type UART_IRQ_Flag is
     (Modem_RI, Modem_CTS, Modem_DCD, Modem_DSR,
      Receive,
      Transmit,
      Receive_Timeout,
      Framing_Error,
      Parity_Error,
      Break_Error,
      Overrun_Error);

   procedure Enable_IRQ (This :        UART_Port;
                         IRQ  :        UART_IRQ_Flag);
   --  Enable the given IRQ flag

   procedure Disable_IRQ (This :        UART_Port;
                          IRQ  :        UART_IRQ_Flag);
   --  Disable the given IRQ flag

   procedure Clear_IRQ (This :        UART_Port;
                        IRQ  :        UART_IRQ_Flag);
   --  Clear the given IRQ flag

   procedure Masked_IRQ_Status (This : UART_Port;
                               IRQ  : UART_IRQ_Flag;
                               Result : out Boolean);
   --  Return true if the given IRQ flag is signaled and enabled

   procedure Raw_IRQ_Status (This : UART_Port;
                            IRQ  : UART_IRQ_Flag;
                            Result : out Boolean);
   --  Return true if the given IRQ flag is signaled even if the flag is not
   --  enabled.

private

   UART_Fraction : constant := 1.0 / 2 ** UARTFBRD_BAUD_DIVFRAC_Field'Size;
   --  type UART_Divider is delta UART_Fraction
   --     range UART_Fraction .. (2.0 ** UARTIBRD_BAUD_DIVINT_Field'Size) - UART_Fraction;

   type UART_Divider_Base is delta UART_Fraction
      range 0.0 .. (2.0 ** UARTIBRD_BAUD_DIVINT_Field'Size) - UART_Fraction;
   subtype UART_Divider is UART_Divider_Base
      range UART_Fraction .. UART_Divider_Base'Last;

   function Div_Integer
     (D : UART_Divider)
       return UARTIBRD_BAUD_DIVINT_Field
     with Post => Div_Integer'Result in
       0 .. UARTIBRD_BAUD_DIVINT_Field (Natural (UART_Divider'Last) - 1) and then
         D >= UART_Divider_Base (Div_Integer'Result) and then
         D < UART_Divider_Base (Div_Integer'Result) + 1.0;

   function Div_Fraction
      (D : UART_Divider)
      return UARTFBRD_BAUD_DIVFRAC_Field;

   function Div_Value
      (Int  : UARTIBRD_BAUD_DIVINT_Field;
       Frac : UARTFBRD_BAUD_DIVFRAC_Field)
       return UART_Divider
     with Pre => not (Int = 0 and Frac = 0);

   for FIFO_IRQ_Level use
     (Lvl_Eighth        => 2#000#,
      Lvl_Quarter       => 2#001#,
      Lvl_Half          => 2#010#,
      Lvl_Three_Quarter => 2#011#,
      Lvl_Seven_Eighth  => 2#100#);

   for UART_IRQ_Flag use
     (Modem_RI        => 2#0000_0000_0001#,
      Modem_CTS       => 2#0000_0000_0010#,
      Modem_DCD       => 2#0000_0000_0100#,
      Modem_DSR       => 2#0000_0000_1000#,
      Receive         => 2#0000_0001_0000#,
      Transmit        => 2#0000_0010_0000#,
      Receive_Timeout => 2#0000_0100_0000#,
      Framing_Error   => 2#0000_1000_0000#,
      Parity_Error    => 2#0001_0000_0000#,
      Break_Error     => 2#0010_0000_0000#,
      Overrun_Error   => 2#0100_0000_0000#);

end RP.UART;
