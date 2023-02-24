function ntLoaded(){
    NetworkTables.addGlobalListener((key, value, isNew)=>{
        if (isNew){
            console.log(key," ", value);
        }
    },true)

        NetworkTables.addKeyListener('/SmartDashboard/leftStick', (key, value, isNew)=>{},true);
        NetworkTables.addKeyListener('/SmartDashboard/rightStick', (key, value, isNew)=>{},true);
    

    //code for the buttons to toggle between pis
    const cameraPorts = [1183,1181];
    let currentCamera = 0;

    const piProtocols=[22,20,16];
    let currentPi = 0;

    function toggleCam() {
        currentCamera=(currentCamera+1)%cameraPorts.length;
        const port=cameraPorts[currentCamera];
        const piIp=piProtocols[currentPi];
        setCameraPort(piIp, port);
    }

    function togglePi(){
        currentPi=(currentPi+1)%piProtocols.length;
        const piIp=piProtocols[currentPi];
        const port=cameraPorts[currentCamera];
        setCameraPort(piIp, port);
    }

    function setCameraPort(piIp, port){
        const url=`http://10.20.51.${piIp}:${port}/stream.mjpg?1674930762001`;
        document.getElementById ("cameraOut").src=url;
    }

    setCameraPort(piProtocols[0], cameraPorts[0]);

    //////canvas element to draw over the camera//////
    var canvas=document.getElementById("canvas");
    var ctx=canvas.getContext("2d");

        ctx.moveTo(150,350);
        ctx.lineTo(207,175);
        ctx.strokeStyle="#0FF"
        ctx.stroke();

        ctx.moveTo(460,350);
        ctx.lineTo(415,175);
        ctx.stroke();

    /////Audio/////Audio/////Audio/////Audio/////Audio/////Audio/////
    JSSoundTest = document.getElementById("soundTest");
    NetworkTables.addKeyListener('/SmartDashboard/State:', (key, value, isNew)=>{
            JSSoundTest.currentTime = 0;
            JSSoundTest.play()
    },true);


    }
