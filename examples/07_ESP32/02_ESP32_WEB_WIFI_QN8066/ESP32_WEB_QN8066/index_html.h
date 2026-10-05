#pragma once
const char INDEX_HTML[] PROGMEM = R"QNHTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>QN8066 FM Transmitter</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: Arial, sans-serif;
      font-size: 14px;
      background: #f0f0f0;
      color: #222;
      min-height: 100vh;
      margin: 0;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    .container {
      width: 100%;
      max-width: 640px;
      margin: 0 auto;
      padding: 4px;
      background: #ffffff;
      box-shadow: 3px 3px 3px rgba(0, 0, 0, 0.05);
    }
    .header {
      background: linear-gradient(45deg, #3a8fc9, #4169E1);
      min-height: 80px;
      display: flex;
      flex-direction: row;
      gap: 6px;
      align-items: center;
      padding: 0 8px;
    }
    .header h1 {
      font-size: 18px;
      font-weight: 600;
      color: #fff;
      margin: 0;
    }
    .settings {
      padding: 16px 8px;
      min-height: 646px;
    }
    .tabs-container {
        display: flex;
        border-bottom: 2px solid #ccc;
    }
    .tab-btn {
        padding: 6px 12px;
        border: none;
        background: transparent;
        cursor: pointer;
        color: #666;
        border-bottom: 2px solid transparent;
        margin-bottom: -2px;
        transition: color 0.15s;
    }
    .tab-btn.active {
        font-weight: 600;
        color: #333;
        border-color: #444;
    }
    .section-heading {
        padding-bottom: 8px;
        margin: 16px 0 8px;
        border-bottom: 1px solid #bbb;
        text-transform: uppercase;
        color: #bbb;
        font-size: 12px;
        font-weight: 600;
    }
    .tab-content {
        padding: 16px 0;
        display: none;
    }
    .tab-content.active-tab {
        display: block;
    }
    .field-row {
        display: flex;
        align-items: center;
        gap: 10px;
        padding: 7px 0;
    }
    /* .field-row:nth-child(odd) {
        background-color: #f0f0f0;
    } */
    .field-label {
        flex: 0 0 180px;
    }
    .field-control {
        flex: 1;
        min-width: 0;
        display: flex;
        flex-direction: row;
    }
    .field-control input[type="text"],
    .field-control select {
        flex: 1;
        padding: 5px 8px;
        border: 1px solid #ccc;
        border-radius: 3px 0 0 3px;
        background: #fff;
        color: #222;
    }
    button {
        color: #fff;
        font-weight: 600;
        border: none;
        cursor: pointer;
        background: #3a8fc9;
    }
    button.set-btn {
        padding: 6px;
        border-radius: 0 3px 3px 0;
    }
    button.submit-btn {
        padding: 6px 12px;
        border-radius: 3px;
    }
    button:hover {
        opacity: 0.75;
    }
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <div>
            <svg width="80px" viewBox="0 0 680 680" xmlns="http://www.w3.org/2000/svg"><g transform="translate(340,340) rotate(45)"><rect fill="none" stroke="white" stroke-width="22" stroke-linecap="round" stroke-linejoin="round" x="-155" y="-155" width="310" height="310" rx="18"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-90" y1="-155" x2="-90" y2="-225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-30" y1="-155" x2="-30" y2="-225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="30" y1="-155" x2="30" y2="-225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="90" y1="-155" x2="90" y2="-225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-90" y1="155" x2="-90" y2="225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-30" y1="155" x2="-30" y2="225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="30" y1="155" x2="30" y2="225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="90" y1="155" x2="90" y2="225"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-155" y1="-90" x2="-225" y2="-90"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-155" y1="-30" x2="-225" y2="-30"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-155" y1="30" x2="-225" y2="30"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="-155" y1="90" x2="-225" y2="90"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="155" y1="-90" x2="225" y2="-90"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="155" y1="-30" x2="225" y2="-30"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="155" y1="30" x2="225" y2="30"/><line stroke="white" stroke-width="22" stroke-linecap="round" x1="155" y1="90" x2="225" y2="90"/></g><polyline fill="none" stroke="white" stroke-width="14" stroke-linecap="round" stroke-linejoin="round" points="288,435 300,400 312,365 322,330 330,300 340,268"/><polyline fill="none" stroke="white" stroke-width="14" stroke-linecap="round" stroke-linejoin="round" points="392,435 380,400 368,365 358,330 350,300 340,268"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="288" y1="435" x2="380" y2="400"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="392" y1="435" x2="300" y2="400"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="300" y1="400" x2="368" y2="365"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="380" y1="400" x2="312" y2="365"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="312" y1="365" x2="358" y2="330"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="368" y1="365" x2="322" y2="330"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="322" y1="330" x2="350" y2="300"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="358" y1="330" x2="330" y2="300"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="330" y1="300" x2="340" y2="268"/><line stroke="white" stroke-width="14" stroke-linecap="round" x1="350" y1="300" x2="340" y2="268"/><path fill="none" stroke="white" stroke-width="16" stroke-linecap="round" d="M308 247A42 42 0 0 0 308 289"/><path fill="none" stroke="white" stroke-width="13" stroke-linecap="round" d="M285 247A66 66 0 0 0 285 289"/><path fill="none" stroke="white" stroke-width="16" stroke-linecap="round" d="M372 247A42 42 0 0 1 372 289"/><path fill="none" stroke="white" stroke-width="13" stroke-linecap="round" d="M395 247A66 66 0 0 1 395 289"/><circle cx="340" cy="268" r="9" fill="white"/></svg>
        </div>
        <h1>QN8066 FM Transmitter</h1>
    </div>
    <div class="settings">
        <div class="tabs-container">
            <button class="tab-btn active" onclick="switchTab('tx')">Transmitter Settings</button>
            <button class="tab-btn" onclick="switchTab('rds')">RDS Settings</button>
        </div>
        <form method="POST" action="/setParameters">
            <div class="tab-content active-tab" id="tab-tx">
                <div class="section-heading">Signal</div>
                <div class="field-row">
                    <span class="field-label">Frequency (MHz):</span>
                    <span class="field-control">
                        <input type="text" id="frequency" name="frequency" maxlength="6" placeholder="e.g. 106.9">
                        <button type="button" class="set-btn" onclick="sendData('frequency')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Power:</span>
                    <span class="field-control">
                    <input type="text" id="power" name="power" maxlength="3">
                    <button type="button" class="set-btn" onclick="sendData('power')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Stereo / Mono:</span>
                    <span class="field-control">
                    <select id="stereo_mono" name="stereo_mono">
                        <option value="0">Stereo</option>
                        <option value="1">Mono</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('stereo_mono')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Pre-Emphasis:</span>
                    <span class="field-control">
                    <select id="pre_emphasis" name="pre_emphasis">
                        <option value="0" selected>50 µs</option>
                        <option value="1">75 µs</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('pre_emphasis')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">MPX Deviation:</span>
                    <span class="field-control">
                    <select id="frequency_deviation" name="frequency_deviation">
                        <option value="55">50 kHz</option>
                        <option value="74">67 kHz</option>
                        <option value="83" selected>75 kHz</option>
                        <option value="94">85 kHz</option>
                        <option value="111">100 kHz</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('frequency_deviation')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Input Impedance:</span>
                    <span class="field-control">
                    <select id="input_impedance" name="input_impedance">
                        <option value="0">10 k&#x2126;</option>
                        <option value="1" selected>20 k&#x2126;</option>
                        <option value="2">40 k&#x2126;</option>
                        <option value="3">80 k&#x2126;</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('input_impedance')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Buffer Gain:</span>
                    <span class="field-control">
                    <select id="buffer_gain" name="buffer_gain">
                        <option value="0">3 dB</option>
                        <option value="1" selected>6 dB</option>
                        <option value="2">9 dB</option>
                        <option value="3">12 dB</option>
                        <option value="4">15 dB</option>
                        <option value="5">18 dB</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('buffer_gain')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Soft Clip:</span>
                    <span class="field-control">
                    <select id="soft_clip" name="soft_clip">
                        <option value="0" selected>Disabled</option>
                        <option value="1">Enabled</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('soft_clip')">&rarr;</button>
                    </span>
                </div>
                <div class="section-heading">Advanced</div>
                <div class="field-row">
                    <span class="field-label">Raw RDS Mode (Serial):</span>
                    <span class="field-control">
                    <select id="raw_mode" name="raw_mode">
                        <option value="0" selected>Disabled</option>
                        <option value="1">Enabled</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('raw_mode')">&rarr;</button>
                    </span>
                </div>
            </div>
            <div class="tab-content" id="tab-rds">
                <div class="section-heading">Global RDS options</div>
                <div class="field-row">
                    <span class="field-label">Data Set Number (DSN):</span>
                    <span class="field-control">
                        <input type="text" id="rds_dsn" name="rds_dsn" maxlength="3" placeholder="0-255">
                        <button type="button" class="set-btn" onclick="sendData('rds_dsn')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Programme Service Number (PSN):</span>
                    <span class="field-control">
                        <input type="text" id="rds_psn" name="rds_psn" maxlength="3" placeholder="0-255">
                        <button type="button" class="set-btn" onclick="sendData('rds_psn')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">UECP Site Address(es):</span>
                    <span class="field-control">
                        <input type="text" id="uecp_site" name="uecp_site" maxlength="15" placeholder="0-3FF hex, comma list up to 4, 0=all" value="0">
                        <button type="button" class="set-btn" onclick="sendData('uecp_site')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">UECP Encoder Address(es):</span>
                    <span class="field-control">
                        <input type="text" id="uecp_enc" name="uecp_enc" maxlength="23" placeholder="0-3F hex, comma list up to 8, 0=all" value="0">
                        <button type="button" class="set-btn" onclick="sendData('uecp_enc')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Add ODA Mapping (AID:Group):</span>
                    <span class="field-control">
                        <input type="text" id="oda_add" name="oda_add" maxlength="16" placeholder="e.g. CD46:8A">
                        <button type="button" class="set-btn" onclick="sendData('oda_add')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Group Sequence (replaces all):</span>
                    <span class="field-control">
                        <input type="text" id="rds_seq" name="rds_seq" maxlength="500" placeholder="e.g. 0A,1A,2A,0A,3A,2A">
                        <button type="button" class="set-btn" onclick="sendData('rds_seq')">&rarr;</button>
                    </span>
                </div>
                <div class="section-heading">RDS Data</div>
                <div class="field-row">
                    <span class="field-label">PI Code:</span>
                    <span class="field-control">
                        <input type="text" id="rds_pi" name="rds_pi" maxlength="4" placeholder="e.g. 819B">
                        <button type="button" class="set-btn" onclick="sendData('rds_pi')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">PIN (hex):</span>
                    <span class="field-control">
                        <input type="text" id="rds_pin" name="rds_pin" maxlength="4" placeholder="e.g. 0000">
                        <button type="button" class="set-btn" onclick="sendData('rds_pin')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">PTY:</span>
                    <span class="field-control">
                        <select id="rds_pty" name="rds_pty">
                            <option value="0" selected>0 – None</option>
                            <option value="1">1 – News</option>
                            <option value="2">2 – Current Affairs</option>
                            <option value="3">3 – Information</option>
                            <option value="4">4 – Sport</option>
                            <option value="5">5 – Education</option>
                            <option value="6">6 – Drama</option>
                            <option value="7">7 – Culture</option>
                            <option value="8">8 – Science</option>
                            <option value="9">9 – Variable</option>
                            <option value="10">10 – Pop Music</option>
                            <option value="11">11 – Rock Music</option>
                            <option value="12">12 – Easy Listening</option>
                            <option value="13">13 – Light Classical</option>
                            <option value="14">14 – Serious Classical</option>
                            <option value="15">15 – Other Music</option>
                            <option value="16">16 – Weather</option>
                            <option value="17">17 – Finance</option>
                            <option value="18">18 – Children's Programmes</option>
                            <option value="19">19 – Social Affairs</option>
                            <option value="20">20 – Religion</option>
                            <option value="21">21 – Phone-in Talk</option>
                            <option value="22">22 – Travel</option>
                            <option value="23">23 – Leisure</option>
                            <option value="24">24 – Jazz Music</option>
                            <option value="25">25 – Country Music</option>
                            <option value="26">26 – National Music</option>
                            <option value="27">27 – Oldies Music</option>
                            <option value="28">28 – Folk Music</option>
                            <option value="29">29 – Documentary</option>
                            <option value="30">30 – Alarm Test</option>
                            <option value="31">31 – Alarm</option>
                        </select>
                        <button type="button" class="set-btn" onclick="sendData('rds_pty')">&rarr;</button>
                    </span>
                </div>
                <div class="section-heading">Flags</div>
                <div class="field-row">
                    <span class="field-label">Traffic Announcement (TA):</span>
                    <span class="field-control">
                    <select id="rds_ta" name="rds_ta">
                        <option value="0">Off</option>
                        <option value="1" selected>On</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_ta')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Traffic Programme (TP):</span>
                    <span class="field-control">
                    <select id="rds_tp" name="rds_tp">
                        <option value="0" selected>Off</option>
                        <option value="1">On</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_tp')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">Music / Speech (MS):</span>
                    <span class="field-control">
                    <select id="rds_ms" name="rds_ms">
                        <option value="0">Speech</option>
                        <option value="1" selected>Music</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_ms')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">DI: Mono/Stereo:</span>
                    <span class="field-control">
                    <select id="rds_di_stereo" name="rds_di_stereo">
                        <option value="0">Mono</option>
                        <option value="1" selected>Stereo</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_di_stereo')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">DI: Artificial Head:</span>
                    <span class="field-control">
                    <select id="rds_di_artifhead" name="rds_di_artifhead">
                        <option value="0" selected>No</option>
                        <option value="1">Yes</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_di_artifhead')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">DI: Compressed:</span>
                    <span class="field-control">
                    <select id="rds_di_compressed" name="rds_di_compressed">
                        <option value="0" selected>No</option>
                        <option value="1">Yes</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_di_compressed')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">DI: Dynamic PTY:</span>
                    <span class="field-control">
                    <select id="rds_ptyi" name="rds_ptyi">
                        <option value="0" selected>Static</option>
                        <option value="1">Dynamic</option>
                    </select>
                    <button type="button" class="set-btn" onclick="sendData('rds_ptyi')">&rarr;</button>
                    </span>
                </div>
                <div class="section-heading">PS / RT</div>
                <div class="field-row">
                    <span class="field-label">PS:</span>
                    <span class="field-control">
                        <input type="text" id="rds_ps" name="rds_ps" maxlength="8" placeholder="8 chars">
                        <button type="button" class="set-btn" onclick="sendData('rds_ps')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">RT:</span>
                    <span class="field-control">
                        <input type="text" id="rds_rt" name="rds_rt" maxlength="64" placeholder="Up to 64 chars">
                        <button type="button" class="set-btn" onclick="sendData('rds_rt')">&rarr;</button>
                    </span>
                </div>
                <div class="field-row">
                    <span class="field-label">RT Repeat Count (0 = loop forever):</span>
                    <span class="field-control">
                        <input type="text" id="rds_rt_repeat" name="rds_rt_repeat" maxlength="3" placeholder="0">
                        <button type="button" class="set-btn" onclick="sendData('rds_rt_repeat')">&rarr;</button>
                    </span>
                </div>
            </div>
            <div class="submit-bar">
                <button type="button" id="submit-btn" class="submit-btn" onclick="submitAll()">Send all &rarr;</button>
            </div>
        </form>
    </div>
</div>
<script>
  function fetchStatus() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/status', true);
    xhr.onreadystatechange = function() {
      if (xhr.readyState == 4 && xhr.status == 200) {
        var s = JSON.parse(xhr.responseText);
        if (s.frequency) document.getElementById('frequency').value = s.frequency;
        ['stereo_mono','pre_emphasis','frequency_deviation','input_impedance',
         'buffer_gain','soft_clip','raw_mode','rds_pty',
         'rds_ta','rds_tp','rds_ms','rds_di_stereo','rds_di_artifhead','rds_di_compressed','rds_ptyi',
         'rds_dsn','rds_psn','rds_rt_repeat','uecp_site','uecp_enc'].forEach(function(id) {
          if (s[id] !== undefined) document.getElementById(id).value = s[id];
        });
        if (s.rds_pi) document.getElementById('rds_pi').value = s.rds_pi.toUpperCase();
        if (s.rds_pin) document.getElementById('rds_pin').value = s.rds_pin.toUpperCase();
        if (s.rds_ps) document.getElementById('rds_ps').value = s.rds_ps.trimEnd();
        if (s.rds_rt) document.getElementById('rds_rt').value = s.rds_rt;
      }
    };
    xhr.send();
  }
  document.addEventListener('DOMContentLoaded', fetchStatus);
  function switchTab(name) {
    document.querySelectorAll('.tab-btn').forEach(function(btn) {
      btn.classList.toggle('active', btn.getAttribute('onclick').indexOf("'" + name + "'") !== -1);
    });
    document.querySelectorAll('.tab-content').forEach(function(p) {
      p.classList.remove('active-tab');
    });
    document.getElementById('tab-' + name).classList.add('active-tab');
  }
  function submitAll() {
    var btn = document.getElementById('submit-btn');
    var form = document.querySelector('form');
    var body = new URLSearchParams(new FormData(form)).toString();
    var xhr = new XMLHttpRequest();
    xhr.open('POST', '/setParameters', true);
    xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
    xhr.onreadystatechange = function() {
      if (xhr.readyState == 4 && xhr.status == 200) {
        btn.textContent = 'Config sent ✓';
        setTimeout(function() { btn.textContent = 'Send all →'; }, 2500);
      }
    };
    xhr.send(body);
  }
  function sendData(fieldId) {
    var value = document.getElementById(fieldId).value;
    var xhr = new XMLHttpRequest();
    xhr.open('POST', '/update', true);
    xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
    xhr.onreadystatechange = function() {
    if (xhr.readyState == 4 && xhr.status == 200) {
        console.log('Response: ' + xhr.responseText);
    }
    };
    xhr.send(fieldId + '=' + encodeURIComponent(value));
  }
</script>
</body>
</html>
)QNHTML";
