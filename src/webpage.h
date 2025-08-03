const char* webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HAMR</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      text-align: center;
      background: linear-gradient(135deg, #32459bff 0%, #764ba2 100%);
      margin: 0;
      padding: 20px;
      color: white;
      min-height: 100vh;
    }

    .container {
      max-width: 500px;
      margin: 0 auto;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 20px;
      padding: 30px;
      backdrop-filter: blur(10px);
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.2);
    }

    h2 {
      margin-top: 0;
      margin-bottom: 30px;
      font-size: 2.2em;
      text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3);
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 15px;
    }

    #joystickZone {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: linear-gradient(145deg, #e0e0e0, #c0c0c0);
      border-radius: 50%;
      position: relative;
      touch-action: none;
      box-shadow: inset 8px 8px 16px rgba(0, 0, 0, 0.2),
                  inset -8px -8px 16px rgba(255, 255, 255, 0.7);
    }

    #stick {
      width: 50px;
      height: 50px;
      background: linear-gradient(145deg, #555, #333);
      border-radius: 50%;
      position: absolute;
      top: 75px;
      left: 75px;
      cursor: pointer;
      transition: all 0.1s ease;
      box-shadow: 4px 4px 8px rgba(0, 0, 0, 0.3),
                  -2px -2px 4px rgba(255, 255, 255, 0.1);
    }

    #stick:hover {
      transform: scale(1.1);
    }

    .control-group {
      margin: 25px 0;
      padding: 20px;
      background: rgba(255, 255, 255, 0.05);
      border-radius: 15px;
      border: 1px solid rgba(255, 255, 255, 0.1);
    }

    .control-group h3 {
      margin: 0 0 15px 0;
      font-size: 1.3em;
      color: #fff;
    }

    .slider-container {
      display: flex;
      align-items: center;
      gap: 15px;
      justify-content: center;
      margin: 15px 0;
    }

    .slider {
      -webkit-appearance: none;
      width: 200px;
      height: 8px;
      border-radius: 4px;
      background: linear-gradient(90deg, #ff6b6b 0%, #4ecdc4 50%, #45b7d1 100%);
      outline: none;
      position: relative;
    }

    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
      transition: all 0.2s ease;
    }

    .slider::-webkit-slider-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 6px 12px rgba(0, 0, 0, 0.4);
    }

    .slider::-moz-range-thumb {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      border: none;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    }

    .slider-labels {
      display: flex;
      justify-content: space-between;
      width: 200px;
      margin: 10px auto 0;
      font-size: 0.9em;
      color: rgba(255, 255, 255, 0.8);
    }

    .slider-value {
      min-width: 60px;
      padding: 8px 12px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 8px;
      font-family: monospace;
      font-size: 1.1em;
      border: 1px solid rgba(255, 255, 255, 0.2);
      margin-top: 10px;
    }

    button {
      padding: 12px 24px;
      font-size: 16px;
      margin: 10px;
      background: linear-gradient(145deg, #4ecdc4, #44a08d);
      color: white;
      border: none;
      border-radius: 25px;
      cursor: pointer;
      transition: all 0.3s ease;
      font-weight: 600;
      box-shadow: 0 4px 15px rgba(68, 160, 141, 0.3);
    }

    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(68, 160, 141, 0.4);
      background: linear-gradient(145deg, #5dd4cc, #4ecdc4);
    }

    button:active {
      transform: translateY(0);
    }

    input[type="number"] {
      width: 100px;
      padding: 12px;
      font-size: 16px;
      margin-right: 10px;
      border: 2px solid rgba(255, 255, 255, 0.2);
      border-radius: 12px;
      background: rgba(255, 255, 255, 0.1);
      color: white;
      text-align: center;
    }

    input[type="number"]::placeholder {
      color: rgba(255, 255, 255, 0.6);
    }

    input[type="number"]:focus {
      outline: none;
      border-color: #4ecdc4;
      box-shadow: 0 0 10px rgba(78, 205, 196, 0.3);
    }

    .status-indicator {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: #4ecdc4;
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0% { opacity: 1; }
      50% { opacity: 0.5; }
      100% { opacity: 1; }
    }

    .info-panel {
      margin-top: 20px;
      padding: 15px;
      background: rgba(0, 0, 0, 0.2);
      border-radius: 10px;
      font-family: monospace;
      font-size: 0.9em;
      text-align: left;
    }

    .trigger-control {
      width: 100%;
      max-width: 300px;
      margin: 0 auto;
    }

    .trigger-end-label {
      font-size: 1.1em;
      font-weight: 600;
      color: rgba(255, 255, 255, 0.9);
      min-width: 30px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>HAMR Robot Control <span class="status-indicator"></span></h2>

    <div class="control-group">
      <h3>Drive Control</h3>
      <div id="joystickZone">
        <div id="stick"></div>
      </div>
    </div>

    <div class="info-panel" id="infoPanel">
      Robot Status: Ready<br>
      Position: X=0.000, Y=0.000, θ=0.0°<br>
      L: 0.00 | R: 0.00<br>
      Last Command: None
    </div>

    <div class="control-group">
      <h3>Turret Control</h3>
      <div class="trigger-control">
        <div class="slider-container">
          <span class="trigger-end-label">L</span>
          <input type="range" id="turretSlider" class="slider" min="-100" max="100" value="0" step="1">
          <span class="trigger-end-label">R</span>
        </div>
        <div class="slider-labels">
          <span>-100%</span>
          <span>0%</span>
          <span>+100%</span>
        </div>
        <div class="slider-value" id="turretValue">0%</div>
      </div>
    </div>

    <div class="control-group">
      <h3>Turret Position</h3>
      <input type="number" id="angleInput" placeholder="Angle (0-360)" min="0" max="360" step="1">
      <button onclick="sendTurretAngle()">Set Position</button>
    </div>

    <div class="control-group">
      <button onclick="resetOdometry()">Reset Odometry</button>
      <button onclick="getPose()">Get Pose</button>
    </div>
  </div>

  <script>
    const stick = document.getElementById('stick');
    const zone = document.getElementById('joystickZone');
    const turretSlider = document.getElementById('turretSlider');
    const turretValue = document.getElementById('turretValue');
    const infoPanel = document.getElementById('infoPanel');
    
    let dragging = false;
    let currentTurretValue = 0;

    // Joystick event listeners
    zone.addEventListener('touchstart', startDrag);
    zone.addEventListener('touchmove', drag);
    zone.addEventListener('touchend', endDrag);
    zone.addEventListener('mousedown', startDrag);
    zone.addEventListener('mousemove', drag);
    zone.addEventListener('mouseup', endDrag);
    zone.addEventListener('mouseleave', endDrag);

    // Turret slider event listeners
    turretSlider.addEventListener('input', function() {
      currentTurretValue = parseInt(this.value);
      turretValue.textContent = currentTurretValue + '%';
      
      if (currentTurretValue < 0) {
        // Left side = L trigger
        const ltValue = Math.abs(currentTurretValue) / 100.0;
        sendTrigger('l', ltValue);
      } else if (currentTurretValue > 0) {
        // Right side = R trigger  
        const rtValue = currentTurretValue / 100.0;
        sendTrigger('r', rtValue);
      } else {
        // Center = stop
        sendTrigger('stop', 0);
      }
      
      updateInfoPanel();
    });

    // Auto-return to center when released
    turretSlider.addEventListener('mouseup', returnToCenter);
    turretSlider.addEventListener('touchend', returnToCenter);

    function returnToCenter() {
      setTimeout(() => {
        if (!turretSlider.matches(':active')) {
          turretSlider.value = 0;
          currentTurretValue = 0;
          turretValue.textContent = '0%';
          sendTrigger('stop', 0);
          updateInfoPanel();
        }
      }, 100);
    }

    function startDrag(e) {
      dragging = true;
      e.preventDefault();
    }

    function drag(e) {
      if (!dragging) return;
      e.preventDefault();
      
      let x = (e.touches ? e.touches[0].clientX : e.clientX) - zone.getBoundingClientRect().left;
      let y = (e.touches ? e.touches[0].clientY : e.clientY) - zone.getBoundingClientRect().top;
      
      let dx = x - 100;
      let dy = y - 100;
      
      let dist = Math.min(Math.sqrt(dx * dx + dy * dy), 75);
      let angle = Math.atan2(dy, dx);
      
      let stickX = Math.cos(angle) * dist + 75;
      let stickY = Math.sin(angle) * dist + 75;
      
      stick.style.left = stickX + 'px';
      stick.style.top = stickY + 'px';
      
      let normX = ((stickX - 75) / 75).toFixed(2);
      let normY = ((stickY - 75) / 75).toFixed(2);
      
      fetch(`/move?x=${normX}&y=${normY}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Drive: X=${normX}, Y=${normY}`);
        })
        .catch(err => console.error('Drive command failed:', err));
    }

    function endDrag() {
      dragging = false;
      stick.style.left = '75px';
      stick.style.top = '75px';
      fetch(`/move?x=0&y=0`)
        .then(() => updateInfoPanel('Drive: Stopped'))
        .catch(err => console.error('Stop command failed:', err));
    }

    function sendTrigger(trigger, value) {
      if (trigger === 'stop') {
        fetch(`/trigger?btn=stop`)
          .then(response => response.text())
          .then(data => {
            console.log('Turret stopped');
          })
          .catch(err => console.error('Stop command failed:', err));
      } else {
        fetch(`/trigger?btn=${trigger}&value=${value}`)
          .then(response => response.text())
          .then(data => {
            console.log(`${trigger.toUpperCase()} set to ${value.toFixed(2)}`);
          })
          .catch(err => console.error(`${trigger} command failed:`, err));
      }
    }

    function sendTurretAngle() {
      const angle = document.getElementById('angleInput').value;
      if (angle === '') {
        alert('Please enter an angle value');
        return;
      }
      
      fetch(`/setTurretAngle?angle=${angle}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Turret angle set to ${angle}°`);
        })
        .catch(err => {
          console.error('Turret angle command failed:', err);
          updateInfoPanel('Turret angle command failed');
        });
    }

    function resetOdometry() {
      fetch('/reset')
        .then(response => response.text())
        .then(data => {
          updateInfoPanel('Odometry reset successfully');
          getPose();
        })
        .catch(err => {
          console.error('Reset failed:', err);
          updateInfoPanel('Reset command failed');
        });
    }

    function getPose() {
      fetch('/pose')
        .then(response => response.json())
        .then(data => {
          updateInfoPanel(`Position: X=${data.x.toFixed(3)}, Y=${data.y.toFixed(3)}, θ=${(data.theta * 180 / Math.PI).toFixed(1)}°`);
        })
        .catch(err => {
          console.error('Get pose failed:', err);
          updateInfoPanel('Failed to get pose data');
        });
    }

    function updateInfoPanel(message) {
      const timestamp = new Date().toLocaleTimeString();
      let triggerStatus = 'None';
      
      if (currentTurretValue < 0) {
        triggerStatus = `L: ${(Math.abs(currentTurretValue) / 100).toFixed(2)}`;
      } else if (currentTurretValue > 0) {
        triggerStatus = `R: ${(currentTurretValue / 100).toFixed(2)}`;
      }
      
      infoPanel.innerHTML = `
        Robot Status: Active<br>
        Turret Control: ${triggerStatus}<br>
        Last Command: ${message || 'None'}<br>
        Time: ${timestamp}
      `;
    }

    // Initialize display
    window.addEventListener('load', function() {
      turretSlider.value = 0;
      turretValue.textContent = '0%';
      updateInfoPanel('System initialized');
    });

    // Auto-update pose every 1 seconds
    setInterval(getPose, 1000);
  </script>
</body>
</html>
)rawliteral";