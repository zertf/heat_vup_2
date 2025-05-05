/*
//Список функций, которые я верменно (или постоянно) убрал сюда, чтобы код читатлся легче и грузился быстрее
//Пока не реализован MF code


/before loop and setup

void relay_params (float temperature) 
{
  if (PIDregulator.Compute()) {
    Input = temperature; //Стоит изменть переменную температуры при надобности
    
    if (Output != 0 && Output != 255) {period = max((20 / (Output/255)), (20 / (1 - (Output/255))));}
    else {period = time_tc;}

    Solid_relay.setPeriod(period); //Из-за особенности библиотеки сначала период, потом выход
    Solid_relay.setPWM(Output);
  }  
}

/setup


if (Display_st) { //Задание начального текста на экране
    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("T pod");
    lcd.setCursor(0, 1);
    lcd.print("T nag");
    lcd.setCursor(0, 2);
    lcd.print("T goal ");
    lcd.print((goal_t));
    //lcd.setCursor(0, 3);
    lcd.setCursor(11, 2);
    lcd.print(" Heat Off");
    /*lcd.setCursor(0, 3);
    lcd.print("Leak: ");
    lcd.print("No");
    }
  
//loop

 

  //Функция проверки реле
  if ((Status == On) or (Status == Done)) {regulator_check();}
  else {digitalWrite(Relay_pin, LOW);} //Срабатывает по таймеру и условию, чтобы передать температуру и изменить состояние реле 

  if (tmr_print.ready(1) and Display_st) {print_display();} //Вывод информации на lcd дисплей

*/