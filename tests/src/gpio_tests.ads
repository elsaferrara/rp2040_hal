--
--  Copyright (C) 2022 Jeremy Grosser <jeremy@synack.me>
--
--  SPDX-License-Identifier: BSD-3-Clause
--
with AUnit.Test_Cases;
with AUnit;

package GPIO_Tests is

   type GPIO_Test is new AUnit.Test_Cases.Test_Case with null record;

   overriding
   procedure Register_Tests
      (T : in out GPIO_Test);

   overriding
   function Name
      (T : GPIO_Test)
      return AUnit.Message_String;

   procedure Test_Configure
      (T : in out AUnit.Test_Cases.Test_Case'Class);

   procedure Test_HAL
      (T : in out AUnit.Test_Cases.Test_Case'Class);

end GPIO_Tests;
