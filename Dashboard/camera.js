console.log("javaScrript has loaded");

toggleCam = () => console.log("toggleCam");
togglePi = () => console.log("togglePi");
camerasource = "";
var funcCycle = 0;

function cameraInit() {

    //code for the button to toggle between cams
    const cameraPorts = [1183, 1181];
    let currentCamera = 0;

    dataTest = () => {
        NetworkTables.putValue("/SmartDashboard/camera/testVal", funcCycle);
        if (funcCycle < 2) {
            funcCycle++;
        }
        else {
            funcCycle = 0;
        }
    }

    NetworkTables.addKeyListener("/SmartDashboard/camera/testVal", (key, value, isNew) => {
        cameras[value]()
    }, true);

    //22 is Shaolin Hand and 16 is Drill Handlespo Traffic Light (testing)
    // 18 is trauma and 20 is Dumb
    const piProtocols = [22, 20, 16, 18];
    let currentPi = 0;

    frontCam = () => {
        const piIp = piProtocols[3];
        const port = cameraPorts[0];
        setCameraPort(piIp, port);
        document.getElementById("cam").innerHTML = "front";
    }

    //if protocol/port need changed tweak url in ntLoaded
    armCam = () => {
        const piIp = piProtocols[3];
        const port = cameraPorts[1];
        setCameraPort(piIp, port);
        document.getElementById("cam").innerHTML = "arm";
    }

    backCam = () => {
        const piIp = piProtocols[1];
        const port = cameraPorts[0];
        setCameraPort(piIp, port);
        document.getElementById("cam").innerHTML = "back";
    }

    var cameras = [frontCam, armCam, backCam];

    function setCameraPort(piIp, port) {
        const url = `http://10.20.51.${piIp}:${port}/stream.mjpg?1674930762001`;
        camerasource = url;
        document.getElementById("cameraOut").src = url;
    }

    setCameraPort(piProtocols[3], cameraPorts[1]);
}

//refreshes camera if not connected
setTimeout(() => {
    document.getElementById("cameraOut").addEventListener("error", () => {
        setTimeout(() => {
            document.getElementById("cameraOut").src = camerasource;
        }, 5000);

    });
    document.getElementById("cameraOut").src = camerasource;
}, 1000);


//temporary to make drawing faster
function locateCursor(event) {
    console.log("x = ", event.clientX);
    console.log("y = ", event.clientY);
}

function ntLoaded() {
    console.log("network tables");
    //filp camera output when the arm swiches sides && arm camera is selected
    NetworkTables.addKeyListener("/SmartDashboard/arm potentiometer", (key, value, isNew) => {
        if (value < 180 && document.getElementById("cameraOut").src == "http://10.20.51.18:1181/stream.mjpg?1674930762001") {
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
        ctx.lineTo(100, value * 100);
        ctx.strokeStyle = "#0FF";
        ctx.stroke();
    }, true)
}