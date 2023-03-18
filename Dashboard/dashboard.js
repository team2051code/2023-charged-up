console.log("javaScrript has loaded");


function ntLoaded() {

    var isTeleop = false;
    var soundPlayed = false;
    var timeTrip = false;

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

    //mode and gear at top of dashboard
    NetworkTables.addKeyListener("/SmartDashboard/drop off mode", (key, value, isNew) => {
        if(value) document.getElementById("mode").innerText = "Drop Off";
        else document.getElementById("mode").innerText = "Pick Up"
    }, true);

    NetworkTables.addKeyListener("/SmartDashboard/Gear", (key, value, isNew) => {
        document.getElementById("gear").innerText = value ? "High" : "Low";
    }, true)

    //audio cues with time //
    NetworkTables.addKeyListener("/SmartDashboard/Time", (key, value, isNew) => {
        document.getElementById("time").innerText = value.toFixed(2)

        value = Math.floor(value)
        
        if (value == -1 && !timeTrip){
            console.log(isTeleop = !isTeleop);
            timeTrip = true
        }        

        if (value % 5 == 0 && value <= 30 && !soundPlayed){
                console.log('audio');
                document.getElementById("soundTest").play();  
                soundPlayed = true;          
        }
        else if (value % 5 == 4) {
            soundPlayed = false;
        }
    }, true)

}
