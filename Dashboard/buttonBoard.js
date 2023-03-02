console.log("Hello, World!");

//ntLoaded is required for the websocket to open with the page
function ntLoaded() {
    console.log("network tables");  
}    

function buttonReset() {
    NetworkTables.putValue("/SmartDashboard/boardButton", 0);
}

function sendButton(ID){
    console.log(ID);
    console.log(NetworkTables.isWsConnected());
    console.log(NetworkTables.putValue("/SmartDashboard/boardButton", ID));
    setTimeout(buttonReset, 1000);
}
