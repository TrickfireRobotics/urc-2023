// Create ros object to communicate over your Rosbridge connection
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

// When the Rosbridge server connects, fill the span with id “status" with “successful"
ros.on('connection', () => {
  document.getElementById("status").innerHTML = "successful";
});

// When the Rosbridge server experiences an error, fill the “status" span with the returned error
ros.on('error', (error) => {
  document.getElementById("status").innerHTML = `errored out (${error})`;
});

// When the Rosbridge server shuts down, fill the “status" span with “closed"
ros.on('close', () => {
  document.getElementById("status").innerHTML = "closed";
});

//listens to mission_control_py
const latency_listener = new ROSLIB.Topic({
  ros,
  name : "/latency",
  messageType : "std_msgs/String"
});
// When we receive a message on /my_topic, add its data as a list item to the “messages" ul
latency_listener.subscribe((message) => {
  let currTime = Date.now();
  let start = message.data.indexOf("=")
  let end = message.data.indexOf(",")
  let rosTime = message.data.substring(start +1,end)
  let decimalPlaces = 1;
  rosTime = rosTime.substring(0,currTime.toString().length);
  let latency =currTime-rosTime;
  console.log(`${latency}ms`);
  //trimming string
  
  document.getElementById("latency").innerHTML = `${latency}ms` ;

  // const newMessage = document. createElement("li");
  // newMessage. appendChild(document. createTextNode(message.data));
});
//controller stuff
window.addEventListener("gamepadconnected", (e) => {
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});

const move_left_drivebase_side = new ROSLIB.Topic({
  label: 'float[2] containing left controler stick',
  ros : ros,
  name : "/move_right_drivebase_side",
  messageType : "std_msgs/msg/Float32MultiArray"
});
const move_right_drivebase_side = new ROSLIB.Topic({
  label: 'float[2] containing right controler stick',
  ros : ros,
  name : "/move_right_drivebase_side",
  messageType : "std_msgs/msg/Float32MultiArray"
});
setInterval(() => {
  const myGamepad = navigator.getGamepads()[0]; // use the first gamepad
  console.log(`Left stick at (${myGamepad.axes[0]}, ${myGamepad.axes[1]})` );
  console.log(`Right stick at (${myGamepad.axes[2]}, ${myGamepad.axes[3]})` );
  let move_left_drivebase_side_data = [myGamepad.axes[0],myGamepad.axes[1]]
  let move_right_drivebase_side_data = [myGamepad.axes[1],myGamepad.axes[2]]
  document.getElementById("left-stick").innerHTML = `[${move_left_drivebase_side_data}]`;
  document.getElementById("right-stick").innerHTML = `[${move_right_drivebase_side_data}]` ;
  
  let move_left_drivebase_side_message = new ROSLIB.Message({
    data: move_left_drivebase_side_data});
  let move_right_drivebase_side_message = new ROSLIB.Message({
    data: move_right_drivebase_side_data});
  move_left_drivebase_side.publish(move_left_drivebase_side_message);
  move_right_drivebase_side.publish(move_right_drivebase_side_message);

}, 500) // 2 times per second
