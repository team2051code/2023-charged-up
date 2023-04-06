console.log("javaScrript has loaded");


function ntLoaded() {

    var isTeleop = false;
    var soundPlayed = false;
    var timeTrip = false;
    var bkgOverride = false;

    console.log("Network tables has loaded");

    NetworkTables.addGlobalListener((key, value, isNew) => {
        if (isNew) {
            console.log(key, " ", value);
        }
    }, true)
 
    //background color of alliance
    NetworkTables.addKeyListener('/FMSInfo/IsRedAlliance', (key, value, isNew) => { 
        if (value && !bkgOverride){
            document.getElementById("teamColor").style.backgroundColor = "#500";
        }
        else if (!value && !bkgOverride){
            document.getElementById("teamColor").style.backgroundColor = "#004";
        }
    }, true);

    //mode and gear at top of dashboard
    NetworkTables.addKeyListener("/SmartDashboard/drop off mode", (key, value, isNew) => {
        if(value) document.getElementById("mode").innerText = "Drop Off";
        else document.getElementById("mode").innerText = "Pick Up";
    }, true);

    // Auto mode
    
     NetworkTables.addKeyListener('/SmartDashboard/autoname', (key, value, isNew) => {
       const AUTO_NAMES = ["place/drive straight", "stop", "autobalance"];
       const nameString = value > AUTO_NAMES.length ? value : AUTO_NAMES[value-1];
       document.getElementById("auto-mode").innerText = nameString;
    }, true);
    
    NetworkTables.addKeyListener("/SmartDashboard/Gear", (key, value, isNew) => {
        document.getElementById("gear").innerText = value ? "High" : "Low";
    }, true);

    //audio cues with time //
    NetworkTables.addKeyListener("/SmartDashboard/Time", (key, value, isNew) => {
        document.getElementById("time").innerText = value.toFixed(2); //converts to string adding likley to concatonate

        value = Math.floor(value);
        
        if (value == -1 && !timeTrip){
            console.log(isTeleop = !isTeleop);
            timeTrip = true;
        }        

        if (value % 5 == 0 && value <= 30 && !soundPlayed){
            console.log('audio');
            document.getElementById("soundTest").play();  
            soundPlayed = true;          
        }
        else if (value % 5 == 0 && value <= 64 && value >= 56  && !soundPlayed){
            console.log('audio');
            document.getElementById("soundTest").play();  
            soundPlayed = true;          
        }
        else if (value % 5 == 0 && value <= 94 && value >= 86  && !soundPlayed){
            console.log('audio');
            document.getElementById("soundTest").play();  
            soundPlayed = true;          
        }
        else if (value % 5 == 0 && value <= 124 && value >= 116  && !soundPlayed){
            console.log('audio');
            document.getElementById("soundTest").play();  
            soundPlayed = true;          
        }
        else if (value % 5 == 4) {
            soundPlayed = false;
        }
    }, true)

    //Auto Listeners/////Auto Listeners/////Auto Listeners///
    var driveLinear = "/SmartDashboard/Commands/DriveLinear";
    autonomous(driveLinear);
    
    var offRamp = "/SmartDashboard/Commands/OffRamp";
    autonomous(offRamp);

    var onRamp = "/SmartDashboard/Commands/OnRamp";
    autonomous(onRamp);

    var areTrue = 0;

    function autonomous(keyName) {
        NetworkTables.addKeyListener(keyName, (key, value, isNew) => {

            value ? areTrue++ : areTrue--;

            if (areTrue > 0) {
                document.getElementById("teleop").style.display = "none";
                document.getElementById("auto").style.display = "block";
                bkgOverride = true;
                document.getElementById("teamColor").style.backgroundColor = "#E4A11B";
            }
            else {
                document.getElementById("teleop").style.display = "block";
                document.getElementById("auto").style.display = "none";
                bkgOverride = false;
            }

        }, true)
    }


    //Y = M * X + B
    //B = -M * -X + Y
    //M = (Y2 - Y1)/(X2 - X1)
    function pointSlope(robotY, expectedY, robotX, expectedX) {
        let slope = (expectedY-robotY)/(expectedX-robotX);
        console.log("slope of", slope);

        let yIntercept = (-slope * - expectedX) + expectedY; 
        console.log("offset of", yIntercept);
    }


    submitData = (valId, keyId) => {
        NetworkTables.putValue(document.getElementById(keyId).innerText, document.getElementById(valId).value - 0);
        console.log((document.getElementById(valId).value - 0), document.getElementById(keyId).innerText);
    }

}
