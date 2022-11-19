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
        background: #02B875;
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
        background-color: #9BC9C7;
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
        <span id="data0">data</span>
        <span id="data1">data</span>
        <table>
          <!-- data table -->
          <tr>
            <!-- data titles -->
            <td style="width:10%">ExhaustTemp(C)</td>
            <td style="width:10%">Engine(RPM)</td>
            <td style="width:10%">Throttle(%)</td>
            <td style="width:10%">Intaketemp(C)</td>
            <td style="width:10%">CoolTemp(C)</td>
          </tr>
          <tr>
            <!-- data values -->
            <td><span id="data2">0</span></td>
            <td><span id="data3">0</span></td>
            
            <td><span id="data5">0</span></td>
            <td><span id="data6">0</span></td>
            <td><span id="data7">0</span></td>
          </tr>
          <tr>
            <!-- data visulize -->
            <td></td>
            <td style="text-align:center">
              <div class="barcontainerV">
                <div class="barV" id="barV1">
                </div>
              </div>
            </td>
            <td></td>
            <td></td>
            <td></td>
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
            if (true) {
              // actual code:         this.readyState == 4 && this.status == 200
              // test code:           true
              const rawDataStr = "10,20,30|40,50,60|500|3000|70|80|25|30|20|";
              // actual code:         this.responseText
              // test code:           "10,20,30|40,50,60|500|3000|70|80|25|30|20|"
              // current data receiving w/ format:
                // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|A1|engineSpeed(RPM)|engineLoad(%)|throttle(%)|intakeTemp(C)|coolantTemp(C)|currentTime(ms)|
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
              dataValues[n0] = singleDataStr;
              n0++;
              singleDataStr = "";
            }
          }
          
          dataValues[2] = ((parseInt(dataValues[2]) * 3.3 / 1024 * 5 / 3.2 - 1.25)/0.005).toFixed(0).toString();
          
          for(var i=0;i<n0;i++){
            var element = document.getElementById("data"+i.toString());
            if(element){
              element.innerHTML = dataValues[i];
            }
          }
          
          //engine RPM max: 13,000
          var enginePercent = (parseFloat(dataValues[3])/13000*100).toString();
          //throttle visulize
          document.getElementById("barV1").style.height = enginePercent+"%";
        }
      </script>
    </body>
  </html>
)=====";
