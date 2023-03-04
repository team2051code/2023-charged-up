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

    NetworkTables.addKeyListener('/FMSInfo/IsRedAlliance', (key, value, isNew) => {
        if (key){
            document.getElementById("teamColor").style.backgroundColor = "#300";
        }
        else {
            document.getElementById("teamColor").style.backgroundColor = "#003"
        }
    }, true)


    //adapt the webpage to account for width reletive to the number of elements in a row
    //IDs need to stay generic because file is shared
    var title1 = ["title0", "title1", "title2", "title3", "title4"];
    var display1 = ["component0", "component1", "component2", "component3", "component4"];

    function WidthAdjust(value, index, array){
        document.getElementById(value).style.left = (window.innerWidth/array.length)*index + "px";
    }

    function adapt(){
        title1.forEach(WidthAdjust);
        display1.forEach(WidthAdjust);
    }     

    //developer console shrinks the window so adapt has to repeat
    setInterval(adapt, 2500);

}
