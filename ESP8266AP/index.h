const char MAIN_page[] PROGMEM = R"=====(
  <!DOCTYPE html>
  <html>
    <style>
      body{
        background-color: #fafafa;
      }
      table, th, td{
        border:1px solid black;
      }
      
      .card{
        max-width: 960px;
        min-height: 540px;
        background: #02b875;
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
        <table>
          <!-- data table -->
          <tr>  
            <!-- data titles -->
            <td style="width:200px">Data0</td>
            <td style="width:100px">Data1</td>
            <td style="width:100px">Data2</td>
          </tr>
          <tr>
            <!-- data values -->
            <td><span id="data0">0</span></td>
            <td><span id="data1">0</span></td>
            <td><span id="data2">0</span></td>
          </tr>
          <tr>
            <!-- data visulize -->
            <td style="text-align:center">
              <div class="GYROcircle">
                <div class="GYROdot" id="GYROdot1">
                </div>
              </div>
            </td>
            <td style="text-align:center">
              <div class="barcontainerV">
                <div class="barV" id="barV1">
                </div>
              </div>
            </td>
            <td>
            </td>
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
              // test code:           "-65,23|80.415|231.54|"
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
          
          for(var i=0;i<n0;i++){
            document.getElementById("data"+i.toString()).innerHTML = dataValues[i];
          }

          // gyro visulize
          // 0-80-160px
          var GyroX=parseFloat(dataValues[0][0])/100*80+80;
          var GyroY=parseFloat(dataValues[0][1])/100*80+80;
          document.getElementById("GYROdot1").style.left = GyroX.toString()+"px";
          document.getElementById("GYROdot1").style.top = GyroY.toString()+"px";

          //throttle visulize
          document.getElementById("barV1").style.height = dataValues[1]+"%";
        }
      </script>
    </body>
  </html>
)=====";
