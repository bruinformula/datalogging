const char MAIN_page[] PROGMEM = R"=====(

<!DOCTYPE html>
<html>
  <head>
    <title>
        Bruin Formula SAE Vehicle Data
    </title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
  <style>
      body{
        background-color: #FAFAFA;
      }
      
      .card > table, th, td{
        border:1px solid black;
        border-color:white;
    border-collapse: collapse;
    padding: 2px;
    background-color: #121212;
    color: #FFF;
      }
      .card{
        max-width: 400px;
        min-height: 520px;
        background: black;
        padding: 20px;
        box-sizing: border-box;
        color: #FAFAFA;
    font-size: 12px;
    font-family: "Verdana";
        margin: 10px auto;
        box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
    text-align: center;
      }
      .barcontainerV{
        background-color: darkslategray;
        position: relative;
        width: 50%;
    max-width: 50px;
        height: 150px;
        display: inline-block;
        margin-top: 4px;
      }
      .barV{
        background-color: whitesmoke;
        position: absolute;
        bottom: 0;
        width: 100%;
        height: 0%;
        box-sizing: border-box;
        transform-origin: bottom;
      }
      .GYROcircle{
        height: 180px;
        width: 180px;
        background-color: #bbb;
        border-radius: 50%;
        display: inline-block;
        position: relative;
        margin-top: 4px;
      }
      .GYROdot{
        height: 20px;
        width: 20px;
        background-color: #FF0000;
        position: absolute;
        top: 80px;
        left: 80px;
        border-radius: 50%;
        display: inline-block;
        text-align: center;
      }
    #barLambdaBlue, #barTargetLambdaYellow{
    margin-top: 50%;
    height: 0%;
    background-color: #05F;
      position: absolute;
      bottom: 50%;
      left: -0px;
    }
    #barLambdaYellow, #barTargetLambdaYellow{
      height: 0%;
      background-color: #BA0;
      position: absolute;
      top: 50%;
      left: -0px;
    }
    #lambdaTable, #lambdaTable tbody, #targetLambdaTable, #targetLambdaTable tbody{
      height: 100%;
      width: 100%;
      border: 0px solid black;
      padding: 0px;
      margin: 0px;
    }
    #lambdaTable tr, #lambdaTable td, #targetLambdaTable tr, #targetLambdaTable td{
      border: 0px solid black;
      padding: 0px;
      margin: 0px;
      background-color: darkslategray;
    }
    </style>
  <body>
    <div class="card">
      <!-- text displayed in the card -->
      <h4>Bruin Formula Mk. 8 Telemetry</h4>
      <p>Log File Name: <span id="data11">No data received yet</span></p>
    <br>
      <!-- <h4 id="rawDataStr">raw data string</h4> -->
      <table>
        <!-- data table -->
        <tr>
          <!-- data titles -->
          <th style="width:10%">Lambda</th>
          <th style="width:10%">Target Lambda</th>
          <th style="width:10%">Engine Speed (RPM)</th>
          <th style="width:10%">Throttle (%)</th>
          <th style="width:10%">Manifold Pressure (kPa)</th>
        </tr>
        <tr style="text-align:right">
          <!-- data values -->
          <td><span id="data8">0</span></td>
          <td><span id="data12">0</span></td>
          <td><span id="data3">0</span></td>
          <td><span id="data5">0</span></td>
          <td><span id="data9">0</span></td>
        </tr>
        <tr style="text-align:center">
          <!-- data visulize -->
          <td>
            <div class="barcontainerV">
        <table id="lambdaTable">
          <tr><td><div class="barV" id="barLambdaBlue"></div></td></tr>
          <tr><td><div class="barV" id="barLambdaYellow"></div></td></tr>
        </table>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
        <table id="targetLambdaTable">
          <tr><td><div class="barV" id="barTargetLambdaBlue"></div></td></tr>
          <tr><td><div class="barV" id="barTargetLambdaYellow"></div></td></tr>
        </table>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barEngRPM"></div>
            </div>
          </td>

          <td>
            <div class="barcontainerV">
              <div class="barV" id="barThro"></div>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barMAP"></div>
            </div>
          </td>
        </tr>
        <tr>
          <!-- data titles -->
          <th style="width:10%">Injector Duty (%)</th>
          <th style="width:10%">Exhaust Temp (C)</th>
          <th style="width:10%">Engine Load (%)</th>
          <th style="width:10%">Intake Temp (C)</th>
          <th style="width:10%">Coolant Temp (C)</th>
        </tr>
        <tr style="text-align:right">
          <!-- data values -->
          <td><span id="data14">0</span></td>
          <td><span id="data2">0</span></td>
          <td><span id="data4">0</span></td>
          <td><span id="data6">0</span></td>
          <td><span id="data7">0</span></td>
        </tr>
        <tr style="text-align:center">
          <!-- data visulize -->
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barInjDuty"></div>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barExTemp"></div>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barEngLoad"></div>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barInTemp"></div>
            </div>
          </td>
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barCoTemp"></div>
            </div>
          </td>
        </tr>
      </table>

    <br><br>
    <table>
      <tr>
        <td width="60px">Fan</td>
        <td width="60px" id="data10">No Data</td>
      </tr>
      <tr>
        <td width="60px">Fuel Pump</td>
        <td width="60px" id="data16">No Data</td>
      </tr>
    </table>
  <br><br>
  <p>Fuel Pressure (psi): <span id="data17">No Data</span></p>
  <p>Battery Voltage: <span id="data15">No Data</span></p>
      <table width="100%">
        <tr>
          <th colspan="3" style="width:50%"> Accelerometer </th>
          <th colspan="3"> Gyroscope </th>
        </tr>
        <tr style="text-align:right">
          <td><span id="ACC0">0</span></td>
          <td><span id="ACC1">0</span></td>
          <td><span id="ACC2">0</span></td>
          <td><span id="GYR0">0</span></td>
          <td><span id="GYR1">0</span></td>
          <td><span id="GYR2">0</span></td>
        </tr>
      </table>
      

    </div>
    <script>
      // JavaScript
      // set refresh rate in Hz
      var refreshRate_Hz = 10;
      // Call this function repetatively with time interval in ms
      setInterval(function() { getData(); }, 1000/refreshRate_Hz);
      // get data from ESP
      
      function getData() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            // actual code:         this.readyState == 4 && this.status == 200
            // test code:           true
            const rawDataStr = this.responseText;
            // actual code:         this.responseText
            // test code:           "10,20,30|40,50,60|500|23,124|42,70|16,80|25|30|105|4,123|0|logFileName.csv|85|128|128|200,200|1|128|"
      // 0: ACC, 1: GYR, 2: EGT, 3: ENGSPD, 4: ENGLD, 5: TPS, 
      // 6: IAT, 7: CLT, 8: O2, 9: MAP, 10: FAN, 11: LOGNAME, 12: TO2, 13: DBTDC, 14: INJDUTY, 15: VBAT, 16: FPUMP, 17: FPR
            // current data receiving:
            // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|exTemp(C)|
            // engineSpeed(RPM)|engineLoad(%)|throttle(%)|
            // intakeTemp(C)|coolantTemp(C)|
            // lambda1|manifold_pressure(kPa)|fan1(bool)|
            // logFileName|
            //document.getElementById("rawDataStr").innerHTML = rawDataStr;
            processData(rawDataStr);
            window.AppInventor.setWebViewString("" + this.responseText);  // RESPUESTA A CadenaDeWebView
          }
        };
        xhttp.open("GET", "readADC", true);
        xhttp.send();
      }
      function processData(allDataStr) {
        dataValues = [];
        var n0 = 0;
        var singleDataStr = "";
        for(var i = 0;i < allDataStr.length;i++) {
          if(allDataStr[i] != '|') {
            singleDataStr += allDataStr[i];
          }
          else {
            var n1 = 0;
            const subDataValues = [];
            var singleSubDataStr = "";
            for(var j = 0;j < singleDataStr.length;j++) {
              if(singleDataStr[j] != ','){
                  singleSubDataStr += singleDataStr[j];
              }
              else{
                  subDataValues[n1]=singleSubDataStr;
                  n1++;
                  singleSubDataStr="";
              }
            }
            subDataValues[n1]=singleSubDataStr;
            dataValues[n0] = subDataValues;
            n0++;
            singleDataStr = "";
          }
        }

    //Data conversions
    //EGT (see adafruit 1178 datasheet)
        dataValues[2] = ((((parseInt(dataValues[2]) * 3.3 / 1024 * 5 / 3.2 - 1.25)/0.005))).toFixed(0).toString();
    //RPM
        dataValues[3] = ((parseInt(dataValues[3][0])*256+parseInt(dataValues[3][1])) * 0.39063).toFixed(0).toString();
    //Engine Load %
        dataValues[4] = ((parseInt(dataValues[4][0])*256+parseInt(dataValues[4][1])) * 0.00261230481157781).toFixed(2).toString();
    //Throttle
        dataValues[5] = ((parseInt(dataValues[5][0])*256+parseInt(dataValues[5][1])) * 0.0015259).toFixed(2).toString();
    //Lambda
        dataValues[8] = (parseInt(dataValues[8]) * 0.00390625 + 0.5).toFixed(2).toString();
    //MAP
        dataValues[9] = ((parseInt(dataValues[9][0])*256+parseInt(dataValues[9][1])) * 0.1).toFixed(1).toString();
    //Fan on/off, string
        dataValues[10] = ((parseInt(dataValues[10])!=0)?"On":"Off");
  //Target Lambda
    dataValues[12] = (parseInt(dataValues[12]) * 0.00390625 + 0.5).toFixed(2).toString();
  //DBTDC
    dataValues[13] = (parseInt(dataValues[13]) * 0.00390625 - 17).toFixed(1).toString();
  //INJDUTY
    dataValues[14] = (parseInt(dataValues[14]) * 0.392157).toFixed(1).toString();
  //VBAT
    dataValues[15] = ((parseInt(dataValues[15][0])*256+parseInt(dataValues[15][1])) * 0.0002455).toFixed(2).toString();
    //Pump on/off, string
        dataValues[16] = ((parseInt(dataValues[16])!=0)?"On":"Off");
  //Fuel Pressure
    dataValues[17] = (parseInt(dataValues[14]) * 0.580151).toFixed(1).toString();
    
        for(var i=2;i<n0;i++){
          var element = document.getElementById("data"+i.toString());
          if(element){
            element.innerHTML = dataValues[i];
          }
        }

        for(var i=0;i<3;i++){
          var eACC = document.getElementById("ACC"+i.toString());
          var eGYR = document.getElementById("GYR"+i.toString());
          if(eACC){
            eACC.innerHTML = dataValues[0][i];
          }
          if(eGYR){
            eGYR.innerHTML = dataValues[1][i];
          }
        }

        //engine RPM max: 13,000
        var enginePercent = (parseFloat(dataValues[3])/13000*100).toString();
        //throttle visulize
        document.getElementById("barEngRPM").style.height = enginePercent+"%";
        
        var throttlePercent = (parseFloat(dataValues[5])).toString();
        document.getElementById("barThro").style.height = throttlePercent+"%";
        
        var engLoadPercent = (parseFloat(dataValues[5])).toString();
        document.getElementById("barEngLoad").style.height = engLoadPercent+"%";
    
    var injDutyPercent = (parseFloat(dataValues[5])).toString();
        document.getElementById("barInjDuty").style.height = injDutyPercent+"%";
        
        
        var exhaustTempPercent = ((parseFloat(dataValues[2])+40)/1040*100).toString();
        var intakeTempPercent = ((parseFloat(dataValues[6])+40)/300*100).toString();
        var coolantTempPercent = ((parseFloat(dataValues[7])+40)/300*100).toString();
        document.getElementById("barExTemp").style.height = exhaustTempPercent+"%";
        document.getElementById("barInTemp").style.height = intakeTempPercent+"%";
        document.getElementById("barCoTemp").style.height = coolantTempPercent+"%";
        //exhaust: -40 to 1000F
        // other: -40 to 260F
    
    var lambdaBluePercent = Math.max(0, dataValues[8]*100 - 100);
    var lambdaYellowPercent = Math.max(0, 100 - dataValues[8]*100);
    document.getElementById("barLambdaBlue").style.height = lambdaBluePercent + "%";
    document.getElementById("barLambdaYellow").style.height = lambdaYellowPercent + "%";
    
    var targetLambdaBluePercent = Math.max(0, dataValues[8]*100 - 100);
    var targetLambdaYellowPercent = Math.max(0, 100 - dataValues[8]*100);
    document.getElementById("barTargetLambdaBlue").style.height = targetLambdaBluePercent + "%";
    document.getElementById("barTargetLambdaYellow").style.height = targetLambdaYellowPercent + "%";
    
    var MAPPercent = dataValues[9]/120 * 100;
    document.getElementById("barMAP").style.height = MAPPercent + "%";

        if(dataValues[10]=="On"){
         document.getElementById("data10").style.backgroundColor = "rgb(0,255,0)";
         document.getElementById("data10").style.color = "black";
        }
        else{
          document.getElementById("data10").style.backgroundColor = "rgb(255,0,0)";
         document.getElementById("data10").style.color = "white";
        }

        if(dataValues[16]=="On"){
         document.getElementById("data16").style.backgroundColor = "rgb(0,255,0)";
         document.getElementById("data16").style.color = "black";
        }
        else{
          document.getElementById("data16").style.backgroundColor = "rgb(255,0,0)";
         document.getElementById("data16").style.color = "white";
        }


      }
    </script>
  </body>
</html>

)=====";
