--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP2040_SVD.Interrupts;
with RP2040_SVD.TIMER;      use RP2040_SVD.TIMER;
with Cortex_M.Hints;
with RP_Interrupts;
with System;

package body RP.Timer.Interrupts with SPARK_Mode is

   procedure IRQ_Handler is
   begin
      TIMER_Periph.INTR.ALARM_2 := True;
   end IRQ_Handler;

   procedure Enable
     (This : in out Delays)
     with SPARK_Mode => Off
   is
      use RP2040_SVD.Interrupts;
      use System;
   begin
      TIMER_Periph.INTE.ALARM_2 := True;

      RP_Interrupts.Attach_Handler
         (Handler => IRQ_Handler'Access,
          Id      => TIMER_IRQ_2_Interrupt,
          Prio    => Interrupt_Priority'First);
   end Enable;

   procedure Disable
      (This : Delays)
   is
   begin
      TIMER_Periph.INTE.ALARM_2 := False;
   end Disable;

   procedure Enabled
      (This : Delays;
       Result : out Boolean)
   is
     Alarm_2 : Boolean := TIMER_Periph.INTE.ALARM_2;
   begin
      Result:= Alarm_2;
      end Enabled;


   procedure Delay_Until
      (This : Delays;
       T    : Time)
   is
      Alarm_2 : Boolean;
      Current_Time : Time;
   begin
      Clock (Current_Time);
      if T <= Current_Time then
         return;
      end if;
      TIMER_Periph.ALARM2 := UInt32 (T and 16#FFFFFFFF#);
      loop
         Cortex_M.Hints.Wait_For_Interrupt;
         Alarm_2 := TIMER_Periph.INTS.ALARM_2;
         Clock (Current_Time);
         if Current_Time >= T then
            return;
         elsif Alarm_2 then
            --  If the high byte of the timer rolled over but we still haven't
            --  reached the target time, reset the alarm and go again
            TIMER_Periph.ALARM2 := UInt32 (T and 16#FFFFFFFF#);
         end if;
      end loop;
   end Delay_Until;

   --  overriding
   procedure Delay_Microseconds
      (Us   : Integer)
   is
      Current_Time : Time;
      Deadline : UInt64;
   begin
      if Us > 0 then
         Clock (Current_Time);
         Deadline := UInt64 (Current_Time) + UInt64 (Us);
         loop
            Clock(Current_Time);
            exit when UInt64 (Current_Time) >= Deadline;
         end loop;
      end if;
   end Delay_Microseconds;

   --  overriding
   procedure Delay_Milliseconds
      (This : Delays;
       Ms   : Natural)
   is
      Current_Time : Time;
   begin
      Clock (Current_Time);
      Delay_Until (This, Current_Time + (1_000 * Time (Ms)));
   end Delay_Milliseconds;

   --  overriding
   procedure Delay_Seconds
      (This : Delays;
       S    : Natural)
   is
      Current_Time : Time;
   begin
      Clock(Current_Time);
      Delay_Until (This, Current_Time + (1_000_000 * Time (S)));
   end Delay_Seconds;
end RP.Timer.Interrupts;
