--
--  Copyright 2021 (C) Jeremy Grosser
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with RP2040_SVD.TIMER; use RP2040_SVD.TIMER;

package body RP.Timer with SPARK_Mode is

   procedure Clock (Result : out Time)
   is
      --  This implementation uses the non-latching TIMERAWH and TIMERAWL
      --  registers in order to be safe for concurrent access.
      Next_High : UInt32;
      High      : UInt32;
      Low       : UInt32;
   begin
      High := TIMER_Periph.TIMERAWH;
      Low := TIMER_Periph.TIMERAWL;
      Next_High := TIMER_Periph.TIMERAWH;

      if Next_High /= High then
         --  If TIMERAWH changed while we were reading TIMERAWL it means that at
         --  some point between the two reads TIMERAWL overflowed and the values
         --  where:
         --    - TIMERAWL = 0
         --    - TIMERAWH = Next_High
         --
         --  These values denote a valid point in time between the call and the
         --  return of this function.

         Low := 0;
         High := Next_High;
      end if;

      Result := Time ((UInt64 (High)) * (2 ** 32) or UInt64 (Low)); -- Shift_Left
   end Clock;

   procedure Busy_Wait_Until (Deadline : Time) is
      DL_High : constant UInt32 :=
        UInt32 ((Deadline - (Deadline mod 2**32)) / 2**32 and 16#FF_FF_FF_FF#);
      DL_Low  : constant UInt32 :=
        UInt32 (Deadline and 16#FF_FF_FF_FF#);

      High : UInt32 := TIMER_Periph.TIMERAWH;
      Low : UInt32;
   begin
      loop
         High := TIMER_Periph.TIMERAWH;
         exit when High >= DL_High;
      end loop;

      Low := TIMER_Periph.TIMERAWL;
      while High = DL_High and then Low < DL_Low loop
         High := TIMER_Periph.TIMERAWH;
      end loop;

   end Busy_Wait_Until;

   function Get_Deadline (T : Time; D : UInt64) return Time
   is (T + Time (D));

   function Milliseconds
     (T : Natural)
       return Time
   is ((Ticks_Per_Second / 1_000) * Time (T));

end RP.Timer;
