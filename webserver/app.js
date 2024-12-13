document.addEventListener('DOMContentLoaded', function (event) {
        var baseHost = document.location.origin
        var streamUrl = baseHost + ':81'
      
        function fetchUrl(url, cb){
          fetch(url)
            .then(function (response) {
              if (response.status !== 200) {
                cb(response.status, response.statusText);
              } else {
                response.text().then(function(data){
                  cb(200, data);
                }).catch(function(err) {
                  cb(-1, err);
                });
              }
            })
            .catch(function(err) {
              cb(-1, err);
            });
        }
      
        function setReg(reg, offset, mask, value, cb){
          //console.log('Set Reg', '0x'+reg.toString(16), offset, '0x'+mask.toString(16), '0x'+value.toString(16), '('+value+')');
          value = (value & mask) << offset;
          mask = mask << offset;
          fetchUrl(`${baseHost}/reg?reg=${reg}&mask=${mask}&val=${value}`, cb);
        }
      
        function getReg(reg, offset, mask, cb){
          mask = mask << offset;
          fetchUrl(`${baseHost}/greg?reg=${reg}&mask=${mask}`, function(code, txt){
            let value = 0;
            if(code == 200){
              value = parseInt(txt);
              value = (value & mask) >> offset;
              txt = ''+value;
            }
            cb(code, txt);
          });
        }
      
        function setXclk(xclk, cb){
          fetchUrl(`${baseHost}/xclk?xclk=${xclk}`, cb);
        }
      
        function setWindow(start_x, start_y, end_x, end_y, offset_x, offset_y, total_x, total_y, output_x, output_y, scaling, binning, cb){
          fetchUrl(`${baseHost}/resolution?sx=${start_x}&sy=${start_y}&ex=${end_x}&ey=${end_y}&offx=${offset_x}&offy=${offset_y}&tx=${total_x}&ty=${total_y}&ox=${output_x}&oy=${output_y}&scale=${scaling}&binning=${binning}`, cb);
        }
      
        const setRegButton = document.getElementById('set-reg')
        setRegButton.onclick = () => {
          let reg = parseInt(document.getElementById('reg-addr').value);
          let mask = parseInt(document.getElementById('reg-mask').value);
          let value = parseInt(document.getElementById('reg-value').value);
      
          setReg(reg, 0, mask, value, function(code, txt){
            if(code != 200){
              alert('Error['+code+']: '+txt);
            }
          });
        }
      
        const getRegButton = document.getElementById('get-reg')
        getRegButton.onclick = () => {
          let reg = parseInt(document.getElementById('get-reg-addr').value);
          let mask = parseInt(document.getElementById('get-reg-mask').value);
          let value = document.getElementById('get-reg-value');
      
          getReg(reg, 0, mask, function(code, txt){
            if(code != 200){
              value.innerHTML = 'Error['+code+']: '+txt;
            } else {
              value.innerHTML = '0x'+parseInt(txt).toString(16)+' ('+txt+')';
            }
          });
        }
      
        const setXclkButton = document.getElementById('set-xclk')
        setXclkButton.onclick = () => {
          let xclk = parseInt(document.getElementById('xclk').value);
      
          setXclk(xclk, function(code, txt){
            if(code != 200){
              alert('Error['+code+']: '+txt);
            }
          });
        }
      
        const setResButton = document.getElementById('set-resolution')
        setResButton.onclick = () => {
          let start_x = parseInt(document.getElementById('start-x').value);
          let offset_x = parseInt(document.getElementById('offset-x').value);
          let offset_y = parseInt(document.getElementById('offset-y').value);
          let total_x = parseInt(document.getElementById('total-x').value);
          let total_y = parseInt(document.getElementById('total-y').value);
          let output_x = parseInt(document.getElementById('output-x').value);
          let output_y = parseInt(document.getElementById('output-y').value);
      
          setWindow(start_x, 0, 0, 0, offset_x, offset_y, total_x, total_y, output_x, output_y, false, false, function(code, txt){
            if(code != 200){
              alert('Error['+code+']: '+txt);
            }
          });
        }
      
        const setRegValue = (el) => {
          let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
          let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
          let mask = el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255;
          let value = 0;
          switch (el.type) {
            case 'checkbox':
              value = el.checked ? mask : 0;
              break;
            case 'range':
            case 'text':
            case 'select-one':
              value = el.value;
              break
            default:
              return;
          }
      
          setReg(reg, offset, mask, value, function(code, txt){
            if(code != 200){
              alert('Error['+code+']: '+txt);
            }
          });
        }
      
        // Attach on change action for register elements
        document
          .querySelectorAll('.reg-action')
          .forEach(el => {
              if (el.type === 'text') {
                  el.onkeyup = function(e){
                      if(e.keyCode == 13){
                          setRegValue(el);
                      }
                  }
              } else {
                  el.onchange = () => setRegValue(el)
              }
          })
      
      
        const updateRegValue = (el, value, updateRemote) => {
          let initialValue;
          let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
          let mask = (el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255) << offset;
          value = (value & mask) >> offset;
          if (el.type === 'checkbox') {
            initialValue = el.checked
            value = !!value
            el.checked = value
          } else {
            initialValue = el.value
            el.value = value
          }
        }
      
      
        const printReg = (el) => {
          let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
          let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
          let mask = el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255;
          let value = 0;
          switch (el.type) {
            case 'checkbox':
              value = el.checked ? mask : 0;
              break;
            case 'range':
            case 'select-one':
              value = el.value;
              break
            default:
              return;
          }
          value = (value & mask) << offset;
          return '0x'+reg.toString(16)+', 0x'+value.toString(16);
        }
      
      
      
        const hide = el => {
          el.classList.add('hidden')
        }
        const show = el => {
          el.classList.remove('hidden')
        }
      
        const disable = el => {
          el.classList.add('disabled')
          el.disabled = true
        }
      
        const enable = el => {
          el.classList.remove('disabled')
          el.disabled = false
        }
      
        const updateValue = (el, value, updateRemote) => {
          updateRemote = updateRemote == null ? true : updateRemote
          let initialValue
          if (el.type === 'checkbox') {
            initialValue = el.checked
            value = !!value
            el.checked = value
          } else {
            initialValue = el.value
            el.value = value
          }
      
          if (updateRemote && initialValue !== value) {
            updateConfig(el);
          } else if(!updateRemote){
            if(el.id === "aec"){
              value ? hide(exposure) : show(exposure)
            } else if(el.id === "agc"){
              if (value) {
                show(gainCeiling)
                hide(agcGain)
              } else {
                hide(gainCeiling)
                show(agcGain)
              }
            } else if(el.id === "awb_gain"){
              value ? show(wb) : hide(wb)
            } else if(el.id === "face_recognize"){
              value ? enable(enrollButton) : disable(enrollButton)
            } else if(el.id == "led_intensity"){
              value > -1 ? show(ledGroup) : hide(ledGroup)
            }
          }
        }
      
        function updateConfig (el) {
          let value
          switch (el.type) {
            case 'checkbox':
              value = el.checked ? 1 : 0
              break
            case 'range':
            case 'select-one':
              value = el.value
              break
            case 'button':
            case 'submit':
              value = '1'
              break
            default:
              return
          }
      
          const query = `${baseHost}/control?var=${el.id}&val=${value}`
      
          fetch(query)
            .then(response => {
              console.log(`request to ${query} finished, status: ${response.status}`)
            })
        }
      
        document
          .querySelectorAll('.close')
          .forEach(el => {
            el.onclick = () => {
              hide(el.parentNode)
            }
          })
      
        // read initial values
        fetch(`${baseHost}/status`)
          .then(function (response) {
            return response.json()
          })
          .then(function (state) {
            document
              .querySelectorAll('.default-action')
              .forEach(el => {
                updateValue(el, state[el.id], false)
              })
            document
              .querySelectorAll('.reg-action')
              .forEach(el => {
                  let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
                  if(reg == 0){
                    return;
                  }
                  updateRegValue(el, state['0x'+reg.toString(16)], false)
              })
          })
      
        const view = document.getElementById('stream')
        const viewContainer = document.getElementById('stream-container')
        const stillButton = document.getElementById('get-still')
        const streamButton = document.getElementById('toggle-stream')
        const enrollButton = document.getElementById('face_enroll')
        const closeButton = document.getElementById('close-stream')
        const saveButton = document.getElementById('save-still')
        const ledGroup = document.getElementById('led-group')
      
        const stopStream = () => {
          window.stop();
          streamButton.innerHTML = 'Start Stream'
        }
      
        const startStream = () => {
          view.src = `${streamUrl}/stream`
          show(viewContainer)
          streamButton.innerHTML = 'Stop Stream'
        }
      
        // Attach actions to buttons
        stillButton.onclick = () => {
          stopStream()
          view.src = `${baseHost}/capture?_cb=${Date.now()}`
          show(viewContainer)
        }
      
        closeButton.onclick = () => {
          stopStream()
          hide(viewContainer)
        }
      
        streamButton.onclick = () => {
          const streamEnabled = streamButton.innerHTML === 'Stop Stream'
          if (streamEnabled) {
            stopStream()
          } else {
            startStream()
          }
        }
      
        enrollButton.onclick = () => {
          updateConfig(enrollButton)
        }
      
        saveButton.onclick = () => {
          var canvas = document.createElement("canvas");
          canvas.width = view.width;
          canvas.height = view.height;
          document.body.appendChild(canvas);
          var context = canvas.getContext('2d');
          context.drawImage(view,0,0);
          try {
            var dataURL = canvas.toDataURL('image/jpeg');
            saveButton.href = dataURL;
            var d = new Date();
            saveButton.download = d.getFullYear() + ("0"+(d.getMonth()+1)).slice(-2) + ("0" + d.getDate()).slice(-2) + ("0" + d.getHours()).slice(-2) + ("0" + d.getMinutes()).slice(-2) + ("0" + d.getSeconds()).slice(-2) + ".jpg";
          } catch (e) {
            console.error(e);
          }
          canvas.parentNode.removeChild(canvas);
        }
      
        // Attach default on change action
        document
          .querySelectorAll('.default-action')
          .forEach(el => {
            el.onchange = () => updateConfig(el)
          })
      
        // Custom actions
        // Gain
        const agc = document.getElementById('agc')
        const agcGain = document.getElementById('agc_gain-group')
        const gainCeiling = document.getElementById('gainceiling-group')
        agc.onchange = () => {
          updateConfig(agc)
          if (agc.checked) {
            show(gainCeiling)
            hide(agcGain)
          } else {
            hide(gainCeiling)
            show(agcGain)
          }
        }
      
        // Exposure
        const aec = document.getElementById('aec')
        const exposure = document.getElementById('aec_value-group')
        aec.onchange = () => {
          updateConfig(aec)
          aec.checked ? hide(exposure) : show(exposure)
        }
      
        // AWB
        const awb = document.getElementById('awb_gain')
        const wb = document.getElementById('wb_mode-group')
        awb.onchange = () => {
          updateConfig(awb)
          awb.checked ? show(wb) : hide(wb)
        }
      
        // Detection and framesize
        const detect = document.getElementById('face_detect')
        const recognize = document.getElementById('face_recognize')
        const framesize = document.getElementById('framesize')
      
        framesize.onchange = () => {
          updateConfig(framesize)
          if (framesize.value > 5) {
            updateValue(detect, false)
            updateValue(recognize, false)
          }
        }
      
        detect.onchange = () => {
          if (framesize.value > 5) {
            alert("Please select CIF or lower resolution before enabling this feature!");
            updateValue(detect, false)
            return;
          }
          updateConfig(detect)
          if (!detect.checked) {
            disable(enrollButton)
            updateValue(recognize, false)
          }
        }
      
        recognize.onchange = () => {
          if (framesize.value > 5) {
            alert("Please select CIF or lower resolution before enabling this feature!");
            updateValue(recognize, false)
            return;
          }
          updateConfig(recognize)
          if (recognize.checked) {
            enable(enrollButton)
            updateValue(detect, true)
          } else {
            disable(enrollButton)
          }
        }
      })
      