with AUnit.Test_Cases;
with AUnit;

package UART_Tests is

   type UART_Test is new AUnit.Test_Cases.Test_Case with null record;

   overriding
   procedure Register_Tests
      (T : in out UART_Test);

   overriding
   function Name
      (T : UART_Test)
      return AUnit.Message_String;

   overriding
   procedure Set_Up
      (T : in out UART_Test);

   procedure Test_Data
      (T : in out AUnit.Test_Cases.Test_Case'Class);

   procedure Test_Break
      (T : in out AUnit.Test_Cases.Test_Case'Class);

end UART_Tests;
