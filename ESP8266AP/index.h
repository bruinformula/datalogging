const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <style>
      body{
        background-color: #FAFAFA;
      }
      
      table, th, td{
        border:1px solid black;
      }
      .card{
        max-width: 960px;
        min-height: 540px;
        background: #0080ff;
        padding: 30px;
        box-sizing: border-box;
        color: #FFF;
        margin:20px;
        box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
      }
      .barcontainerV{
        background-color: #181818;
        position: relative;
        width: 50px;
        height: 320px;
        display: inline-block;
        margin-top: 4px;
      }
      .barV{
        background-color: #17ff59;
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
    </style>
  <body>
    <div class="card">
      <!-- text displayed in the card -->
      <h4>Update web page without refresh</h4>
      <h4 id="rawDataStr">raw data string</h4>
      <table>
        <!-- data table -->
        <tr>
          <!-- data titles -->
          <th style="width:10%">Exhaust Temperature (C)</th>
          <th style="width:10%">Engine (RPM)</th>
          <th style="width:10%">Throttle (%)</th>
          <th style="width:10%">Intake Temperature (C)</th>
          <th style="width:10%">Coolant Temperature (C)</th>
        </tr>
        <tr style="text-align:right">
          <!-- data values -->
          <td><span id="data2">0</span></td>
          <td><span id="data3">0</span></td>
          <td><span id="data5">0</span></td>
          <td><span id="data6">0</span></td>
          <td><span id="data7">0</span></td>
        </tr>
        <tr style="text-align:center">
          <!-- data visulize -->
          <td>
            <div class="barcontainerV">
              <div class="barV" id="barExTemp"></div>
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

      <!--table for extra data w/o visulization-->
      <br/>
      <table width="100%">
        <tr>
          <th colspan="3"> Accelerometer </th>
          <th colspan="3"> Gyroscope </th>
          <th>Engine Load (%)</th>
          <th>Lambda</th>
          <th>Manifold Pressure (kPa)</th>
          <th>Cooling Fan Status</th>
        </tr>
        <tr style="text-align:right">
          <td><span id="ACC0">0</span></td>
          <td><span id="ACC1">0</span></td>
          <td><span id="ACC2">0</span></td>
          <td><span id="GYR0">0</span></td>
          <td><span id="GYR1">0</span></td>
          <td><span id="GYR2">0</span></td>
          <td><span id="data4">0</span></td>
          <td><span id="data8">0</span></td>
          <td><span id="data9">0</span></td>
          <td style="text-align:center" id="fanCell"><span id="data10">0</span></td>
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
            // test code:           "10,20,30|40,50,60|500|23,124|42,70|16,80|25|30|35|4,123|0|"
            // current data receiving:
            // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|exTemp(C)|
            // engineSpeed(RPM)|engineLoad(%)|throttle(%)|
            // intakeTemp(C)|coolantTemp(C)|
            // lambda1|manifold_pressure(kPa)|fan1(bool)|
            document.getElementById("rawDataStr").innerHTML = rawDataStr;
            processData(rawDataStr);
            window.AppInventor.setWebViewString("" + this.responseText);  // RESPUESTA A CadenaDeWebView
          }
        };
        xhttp.open("GET", "readADC", true);
        xhttp.send();
      }
      function processData(allDataStr) {
        const dataValues = [];
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

        // data conversions
        dataValues[2] = ((parseInt(dataValues[2]) * 3.3 / 1024 * 5 / 3.2 - 1.25)/0.005).toFixed(0).toString();
        dataValues[3] = ((parseInt(dataValues[3][0])*256+parseInt(dataValues[3][1])) * 0.39063).toFixed(0).toString();
        dataValues[4] = ((parseInt(dataValues[4][0])*256+parseInt(dataValues[4][1])) * 0.00261230481157781).toFixed(2).toString();
        dataValues[5] = ((parseInt(dataValues[5][0])*256+parseInt(dataValues[5][1])) * 0.0015259).toFixed(2).toString();
        dataValues[9] = ((parseInt(dataValues[9][0])*256+parseInt(dataValues[9][1])) * 0.1).toFixed(0).toString();
        dataValues[10] = ((parseInt(dataValues[10])!=0)?"On":"Off");

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

        if(dataValues[10]=="On"){
         document.getElementById("fanCell").style.backgroundColor = "rgb(0,255,0)";
         document.getElementById("data10").style.color = "black";
        }
        else{
          document.getElementById("fanCell").style.backgroundColor = "rgb(255,0,0)";
         document.getElementById("data10").style.color = "white";
        }


      }
    </script>
  </body>
</html>
)=====";
