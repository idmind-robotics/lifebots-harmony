namespace = 'rovid'
topics = {
    'Batteries': ['/harmony/sensors_node/voltages', 'idmind_msgs/msg/Voltages'],
    'Mission State': ['/harmony/mission/state_json', 'std_msgs/msg/String'],
    'Alarm': ['/harmony/mission/alarm', 'std_msgs/msg/String'],
}

max_bat = 29.0
min_bat = 24.0
pub_topics = {'Velocities': ['/rovid/idmind_webinterface/set_velocities', 'geometry_msgs/Twist']}
subscribers = {};
publishers = {};
version = 0

function display_roscore_msg(msg){
    document.getElementById("roscore_msgs").innerHTML = "<p class='roscore_msg'>Status: "+msg+"</p>"
};

function connect_to_ros(){
    display_roscore_msg("Connecting to ROS...");
    ros = new ROSLIB.Ros({
        // url : 'ws://127.0.0.1:9090'
        url : 'ws://'+window.location.host.split(":")[0]+':9090'
    });

    ros.on('connection', function(){
        display_roscore_msg("Connected to websocket server.");
        start_subscribers();
        start_publishers();
    });

    ros.on('error', function(error) {
        display_roscore_msg('Error connecting to websocket server: '+ error);
    });

    ros.on('close', function() {
        display_roscore_msg('Connection to websocket server closed.');
    });
}

function start_publishers(){
    publishers["alarm_clear"] = new ROSLIB.Topic({
        ros : ros,
        name : "/harmony/people_detection_node/enable_alarm",
        messageType : 'std_msgs/msg/Bool',
        // div_name: name.toLowerCase()
    });
    publishers['task_clear'] = new ROSLIB.Topic({
        ros : ros,
        name : "/harmony/mission/clear_wait_task",
        messageType : 'std_msgs/msg/Bool',
        // div_name: name.toLowerCase()
    });
}

function start_subscribers(){
    body = document.getElementsByTagName("BODY")[0]
    for(var key in topics) {
        var value = topics[key];
        create_subscriber(key, value[0], value[1]);
    }
}

function create_div(name){
    new_div = document.createElement("div");
    new_div.setAttribute("id", name.toLowerCase());
    return new_div;
}

function create_subscriber(name, topic, msg_type){
    // This is risky, because might change div_name
    subscribers[name] = new ROSLIB.Topic({
        ros : ros,
        name : topic,
        messageType : msg_type,
    });
    // update_battery((27-min_bat)/(max_bat-min_bat), (27-min_bat)/(max_bat-min_bat), false);
    // Create if clauses or switch clauses for special cases, like batteries
    // General publishing system
    if (msg_type == "idmind_msgs/msg/Voltages"){
        console.log("Creating subscriber to ", topic, " with ", msg_type);
        // update_battery(0, 0, true);
        subscribers[name].subscribe(function(message){
            // console.log(message);
            ebat = Math.min(Math.max(0, (message.electronic_battery_voltage-min_bat)/(max_bat-min_bat)), 1);
            mbat = Math.min(Math.max(0, (message.motor_battery_voltage-min_bat)/(max_bat-min_bat)), 1);
            charging = (message.cable_power_voltage > 16)
            // ebat = 0;
            // mbat = 0;
            // charging = true;
            update_battery(ebat, mbat, charging);
        });
    }
    
    if (name == 'Mission State'){
        console.log("Creating subscriber to ", topic, " with ", msg_type);
        subscribers[name].subscribe(function(message){
            console.log("Mission state received - "+message.data);
            const obj = JSON.parse(message.data);
            document.getElementById("mission_info").innerHTML = "<h3>Mission Info</h3>"
            document.getElementById("mission_info").innerHTML += "<p>Mission: "+obj.mission_state+"</p>"
            document.getElementById("mission_info").innerHTML += "<p>Status: "+obj.mission_state_message+"</p>"
        });
    }

    if (name == 'Alarm'){
        console.log("Creating subscriber to ", topic, " with ", msg_type);
        version += 1
        subscribers[name].subscribe(function(message){
            console.log("Alarm received");
            document.getElementById("alarm_panel").style.display = "block";
            document.getElementById("alarm_panel").innerHTML = "<h3>Alarm Detected</h3>";

            document.getElementById("alarm_panel").innerHTML = "<h3>Alarm Detected</h3>";
            document.getElementById("alarm_panel").innerHTML += "<p>Description: Someone has fallen!</p>";
            // document.getElementById("alarm_panel").innerHTML += "<img src='http://"+window.location.host.split(":")[0]+":8080/stream?topic=/harmony/mission/alarm' height='400px' alt='Waiting for Camera Stream'>"
            document.getElementById("alarm_panel").innerHTML += "<img src='img/alarm.jpg?v="+version+"' alt='Waiting for image'>"
            document.getElementById("alarm_panel").innerHTML += "<p id='clear_button'><a onclick='clear_alarm_button_pressed()'>Clear Alarm</a></p>"
        });
    }
}

function rosmsg_to_str(msg_type, message){
    htm_msg = "";
    switch(msg_type){
        case 'std_msgs/String':
        case 'std_msgs/Float32':
        case 'std_msgs/Float64':
        case 'std_msgs/Int8':
        case 'std_msgs/Int16':
        case 'std_msgs/Int32':
        case 'std_msgs/Int64':
            htm_msg = "<p>"+message.data+"</p>";
            break;
        case 'idmind_messages/Voltages':
            htm_msg = "<ul>";
            htm_msg += "<li>Electronic Battery:<ul>";
            htm_msg += "<li>"+message.electronic_battery_voltage+"V</li>";
            htm_msg += "<li>"+message.electronic_battery_current+"A</li>";
            htm_msg += "</ul></li>";
            htm_msg += "<li>Motor Battery:<ul>";
            htm_msg += "<li>"+message.motor_battery_voltage+"V</li>";
            htm_msg += "<li>"+message.motor_battery_current+"A</li>";
            htm_msg += "</ul></li>";
            htm_msg += "<li>Cable: "+message.cable_power_voltage+"V</li>";
            htm_msg += "<li>Motor Status: "+message.motor_power_status+"V</li>";
            htm_msg += "</ul>";
            break;
        default:
            htm_msg = "<p>Unknown message type - "+msg_type+"<p>";
            break;
    }
    return htm_msg;
}

function clear_alarm_button_pressed(){
    console.log("Alarm cleared")
    var disable = new ROSLIB.Message({
        data: false});
    publishers['alarm_clear'].publish(disable);
    var continue_msg = new ROSLIB.Message({
        data: true});
    publishers['task_clear'].publish(continue_msg);
    document.getElementById("alarm_panel").style.display = "none";
}    
