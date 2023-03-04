console.log("javaScrript has loaded");

ctx = undefined;
var isSaved = false;
toggleCam = () => console.log("toggleCam");
togglePi = () => console.log("togglePi");

function cameraInit(){
        
        //code for the buttons to toggle between pis
        const cameraPorts = [1183, 1181];
        let currentCamera = 0;

        const piProtocols = [22, 20, 16];
        let currentPi = 0;

        toggleCam = () => {
            currentCamera = (currentCamera + 1) % cameraPorts.length;
            const port = cameraPorts[currentCamera];
            const piIp = piProtocols[currentPi];
            setCameraPort(piIp, port);
        }

        togglePi = () => {
            currentPi = (currentPi + 1) % piProtocols.length;
            const piIp = piProtocols[currentPi];
            const port = cameraPorts[currentCamera];
            setCameraPort(piIp, port);
        }

        function setCameraPort(piIp, port) {
            const url = `http://10.20.51.${piIp}:${port}/stream.mjpg?1674930762001`;
            document.getElementById("cameraOut").src = url;
        }

        setCameraPort(piProtocols[0], cameraPorts[0]);
        
}

function ntLoaded() {
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