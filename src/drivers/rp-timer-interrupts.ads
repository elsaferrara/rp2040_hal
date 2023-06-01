--
--  Copyright 2022 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with HAL.Time; use HAL.Time;

package RP.Timer.Interrupts with SPARK_Mode, Elaborate_Body is

   type Delays is tagged null record; -- new HAL.Time.Delays with

   procedure Enabled
      (This : Delays;
       Result : out Boolean);

   --  Enables the ALARM2 interrupt used by the Delay methods.
   procedure Enable
      (This : in out Delays);

   procedure Disable
      (This : Delays);

   procedure Delay_Until
      (This : Delays;
       T    : Time);
       --  with Pre'Class => Enabled (This);

   --  Microsecond delays are assumed to be relatively short and are
   --  implemented with a polling loop rather than interrupts
   --  overriding
   procedure Delay_Microseconds
      (Us   : Integer);

   --  overriding
   procedure Delay_Milliseconds
      (This : Delays;
       Ms   : Natural);
     --  with Pre => Enabled (This);


   --  overriding
   procedure Delay_Seconds
      (This : Delays;
       S    : Natural);
       --  with Pre => Enabled (This);

end RP.Timer.Interrupts;
