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

}
