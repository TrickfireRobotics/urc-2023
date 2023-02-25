const testForLatency = (e) => {
  const currTime = Date.now();
  
}



// //controller stuff
window.addEventListener("gamepadconnected", (e) => {
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});
// console.log("HELLO WORLD");


//listens to mission_control_py

const my_topic_listener = new ROSLIB.Topic({
  ros,
  name : "/latency",
  messageType : "std_msgs/String"
});
const move_right_drivebase_side = new ROSLIB.Topic({
  ros,
  name : "/move_right_drivebase_side",
  messageType : "std_msgs/Float32MultiArray"
});
setInterval(() => {
  const myGamepad = navigator.getGamepads()[0]; // use the first gamepad
  let move_right_drivebase_side_data = myGamepad.axes[0];
  console.log(`Left stick at (${myGamepad.axes[0]}, ${myGamepad.axes[1]})` );
  console.log(`Right stick at (${myGamepad.axes[2]}, ${myGamepad.axes[3]})` );
  move_right_drivebase_side.publish("" + move_right_drivebase_side_data);
}, 2000) // print axes 10 times per second

// When we receive a message on /my_topic, add its data as a list item to the “messages" ul
my_topic_listener.subscribe((message) => {
  // let rosTime = message.data.
  let currTime = Date.now();
  // console.log(currTime);
  let start = message.data.indexOf("=")
  let end = message.data.indexOf(",")
  let rosTime = message.data.substring(start +1,end)
  let decimalPlaces = 1;
  rosTime = rosTime.substring(0,currTime.toString().length);
  // rosTime /= 10;
  // rosTime /= (Math.pow(10,decimalPlaces));
  console.log(`${currTime-rosTime}ms`);
  //trimming string
  
  // const ul = document. getElementById("messages");
  // const newMessage = document. createElement("li");
  // newMessage. appendChild(document. createTextNode(message.data));
});