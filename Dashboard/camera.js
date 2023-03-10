console.log("javaScrript has loaded");

toggleCam = () => console.log("toggleCam");
togglePi = () => console.log("togglePi");

function cameraInit(){
        
        //code for the buttons to toggle between pis
        const cameraPorts = [1183, 1181];
        let currentCamera = 0;

        //22 is Shaolin Hand (testing)
        //16 is Drill Handlespo Traffic Light and 20 is Dumb
        //Trauma is unknown find unkown 
        const piProtocols = [22, 20, 16];
        let currentPi = 0;

        frontCam = () => {
            const piIp = piProtocols[1];
            const port = cameraPorts[0];
            setCameraPort(piIp, port);
        }

        //if protocol/port need changed tweak url in ntLoaded
        armCam = () => {
            const piIp = piProtocols[2];
            const port = cameraPorts[0];
            setCameraPort(piIp, port);
        }

        backCam = () => {
            const piIp = piProtocols[1];
            const port = cameraPorts[1];
            setCameraPort(piIp, port);
        }

        function setCameraPort(piIp, port) {
            const url = `http://10.20.51.${piIp}:${port}/stream.mjpg?1674930762001`;
            document.getElementById("cameraOut").src = url;
        }

        setCameraPort(piProtocols[0], cameraPorts[0]);
}

//temporary to make drawing faster
function locateCursor(event){
    console.log("x = ", event.clientX);
    console.log("y = ", event.clientY);
}

function ntLoaded() {
    //filp camera output when the arm swiches sides && arm camera is selected
    NetworkTables.addKeyListener("/SmartDashboard/arm potentiometer", (key, value, isNew) => {
        if(value > 180 && document.getElementById("cameraOut").src == "http://10.20.51.16:1183/stream.mjpg?1674930762001") {
            document.getElementById("cameraOut").style.transform = "rotate(180deg)"
        }
        else {
            document.getElementById("cameraOut").style.transform = "rotate(0deg)"
        }
    }, true);


    NetworkTables.addKeyListener("/SmartDashboard/leftStick", (key, value, isNew) => {
        console.log("network tables")
        var canvas = document.getElementById("canvas");
        ctx = canvas.getContext("2d");

        ctx.beginPath();
        ctx.clearRect(0, 0, 100, 100);
        ctx.moveTo(0, 0);
        ctx.lineTo(100, value*100);
        ctx.strokeStyle = "#0FF";
        ctx.stroke();
    }, true)
}