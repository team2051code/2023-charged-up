console.log("javaScrript has loaded");


function ntLoaded() {

    console.log("Network tables has loaded");

    NetworkTables.addGlobalListener((key, value, isNew) => {
        if (isNew) {
            console.log(key, " ", value);
        }
    }, true)

    /////Audio/////Audio/////Audio/////Audio/////Audio/////Audio/////
    JSSoundTest = document.getElementById("soundTest");
    NetworkTables.addKeyListener('/SmartDashboard/State:', (key, value, isNew) => {
        JSSoundTest.currentTime = 0;
        JSSoundTest.play()
    }, true);

    //background color of alliance
    NetworkTables.addKeyListener('/FMSInfo/IsRedAlliance', (key, value, isNew) => {
        if (key){
            document.getElementById("teamColor").style.backgroundColor = "#300";
        }
        else {
            document.getElementById("teamColor").style.backgroundColor = "#003"
        }
    }, true)

    //display mode at top of dashboard
    NetworkTables.addKeyListener("/SmartDashboard/drop off mode", (key, value, isNew) => {
        if(value) document.getElementById("mode").innerText = "Drop Off";
        else document.getElementById("mode").innerText = "Pick Up"
    }, true);

}
