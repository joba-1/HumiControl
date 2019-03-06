# Example lua script to read json sensor values

* Put the script in /opt/domoticz/scripts/lua_parsers
* Create domoticz hardware device "HTTP/HTTPS poller"
    * use http://192.168.1.139/json for the url (replace 192.168.1.139 with your sensors ip or hostname)
    * As content type use application/json
    * As command use the lua script filename
* Create domoticz devices by selecting "create virtual sensor"
    * use custom type or temp/humi/baro
* Edit the script to replace the domoticz id's with those just created
