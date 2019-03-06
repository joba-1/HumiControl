-- JSON parser handling data from HumiControl firmware

-- Retrieve the request content
s = request['content'];

-- Update some devices (replace indexes 39x with yours)
local m = domoticz_applyJsonPath(s,'monitor.soil_moisture.value')
local h = domoticz_applyJsonPath(s,'monitor.air_humidity.value')
local t = domoticz_applyJsonPath(s,'monitor.temperature.value')
local p = domoticz_applyJsonPath(s,'monitor.pressure.value')
local s = domoticz_applyJsonPath(s,'monitor.speed.value')
if not (m == nil) then
  domoticz_updateDevice(393,'',m)
end
if not (h == nil) then
  domoticz_updateDevice(394,'',h)
end
if not (t == nil) then
  domoticz_updateDevice(395,'',t)
end
if not (p == nil) then
  domoticz_updateDevice(396,'',p)
end
if not (s == nil) then
  domoticz_updateDevice(397,'',s)
end
if not (p == nil or h == nil or t == nil) then
  domoticz_updateDevice(391,'',t .. ";" .. h .. ";0;" .. p .. ";0")
end
