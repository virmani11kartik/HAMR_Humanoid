/*

HTML and Javascript

*/

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
    <html>
    <body>

    <h1>Motor Control Interface</h1>

    <div class="slidecontainer">
    <p>Frequency (-1 to 1): <span id="directionValue">0</span></p>
    <input type="range" id="directionSlider" min="-1" max="1" value="0" step="1" oninput="updateSlider(this.value, 'direction')">
    </div>

    <div class="slidecontainer">
    <p>Duty Cycle (0 to 100%): <span id="dutyValue">50</span>%</p>
    <input type="range" id="dutySlider" min="0" max="100" value="50" oninput="updateSlider(this.value, 'duty')">
    </div>

    <script>
    function updateSlider(value, type) {
      var xhttp = new XMLHttpRequest();
      xhttp.open("GET", "set?type=" + type + "&value=" + value, true);
      xhttp.send();
  
      if(type === 'direction') {
          document.getElementById("directionValue").innerHTML = value;
      } else if(type === 'duty') {
          document.getElementById("dutyValue").innerHTML = value;
      }
    }
    </script>

    </body>
    </html>

)===";
