console.log("javaScrript has loaded");

function ntLoaded() {

    console.log("Network tables has loaded");

    NetworkTables.addGlobalListener((key, value, isNew) => {
        if (isNew) {
            console.log(key, " ", value);
        }
    }, true)

    NetworkTables.addKeyListener('/SmartDashboard/leftStick', (key, value, isNew) => { }, true);
    NetworkTables.addKeyListener('/SmartDashboard/rightStick', (key, value, isNew) => { }, true);

    /////Audio/////Audio/////Audio/////Audio/////Audio/////Audio/////
    JSSoundTest = document.getElementById("soundTest");
    NetworkTables.addKeyListener('/SmartDashboard/State:', (key, value, isNew) => {
        JSSoundTest.currentTime = 0;
        JSSoundTest.play()
    }, true);

    NetworkTables.addKeyListener('/FMSInfo/IsRedAlliance', (key, value, isNew) => {
        if (key){
            document.getElementById("teamColor").style.backgroundColor = "#300";
        }
        else {
            document.getElementById("teamColor").style.backgroundColor = "#003"
        }
    }, true)

    //adapt the webpage to account for width reletive to the number of elements in a row
    //this sucked to make it is also just manual bootstrap

    var title1 = ["enabledT", "pressureT", "brakeT", "setpointT", "voltageT"];
    var display1 = ["enabled", "pressure", "brake", "setpoint", "voltage"];

     function WidthAdjust(value, index, array){
        document.getElementById(value).style.left = window.innerWidth/array.length*index + "px";
     }

     function adapt(){
        title1.forEach(WidthAdjust);
        display1.forEach(WidthAdjust);
     }
     

     //developer console shrinks the window so adapt has to repeat
     setInterval(adapt, 2500);

}
