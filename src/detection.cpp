#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <fcntl.h>
#include <tr1/memory>

// Absolute path to the source folder of the package
std::string path = "/home/summit/catkin_ws/src/new_hardware_detection/src/outputs/";
std::string grep_id = " | grep -oE \"[0-9a-z]{4}:[0-9a-z]{4}\" ";


std::string detect_changes(std::string tool);
std::string first_word(std::string); 
std::string exec(const char*);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main function. Saves the original configuration (if necessary) and assembles the message to the analysis node. Check new hardware components every 2 seconds.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main (int argc, char **argv)
{
	ros::init(argc, argv, "detection");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise <std_msgs::String>("new_devices", 100);
	ros::Rate loop_rate(0.5);

	int count = 0, file = 0;
	
	//Set of all linux tools we want to use for detection of new hardware
	std::string used_tools[10] = {"lsusb" , "lspci -n" , "" , "" , "" , "" , "" , "" , "" , ""};
	std::string cmd;
	std::string file_path, file_name;

	//Message object
	std_msgs::String msg;
	msg.data = "";
	std::string new_components, previous_msg = "";

	//Saving the original configuration. It's represented by lists of IDs from tools
	for (int i=0; used_tools[i] != ""; i++) {

		file_name = first_word(used_tools[i]);
		ROS_INFO("Saving output of %s...", used_tools[i].c_str());
		
		// we don't want to save output, if we already saved it
		file_path = path + file_name + ".origin";
		if (access(file_path.c_str(), F_OK) == 0) {
			ROS_INFO("already exists\n");
			continue;
			}

		// cmd is the command for linux. It should look like this: "lsusb > path/lsusb.origin"
		cmd = used_tools[i] + grep_id + " > " + file_path;
		ROS_DEBUG("Command: %s", cmd.c_str());
		system(cmd.c_str());
		ROS_INFO("done\n");
		}
	
	ROS_INFO("Starting detecting new devices...\n");

	// Detection loop
	while (ros::ok()) {
		loop_rate.sleep();

		for (int i=0; used_tools[i] != ""; i++) {
			if ((new_components = detect_changes (used_tools[i])) != "") {
				msg.data += first_word(used_tools[i]) + "\n" + new_components + "\n";
				
				}
			}
		
		if (msg.data != "" & msg.data != previous_msg) { // if message is not empty and not equal to previous
			chatter_pub.publish(msg);
			ROS_INFO("New components are detected! They look like this: \n%s", msg.data.c_str());
			previous_msg = msg.data;
			count++;
			}
		msg.data = "";

		ros::spinOnce;
	}
	
	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Detection function. Check if every device in current configuration was in the original.
//
// tool - linux tool for monitoring
// returns string with all new IDs from the <tool>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string detect_changes(std::string tool) {
	std::string origin, current, diff = "";
	std::string cmd;

	// Load original configuration from the file
	std::string file_path = path + first_word(tool) + ".origin";
	int file = 0, count;
	char buff[512];
	memset(buff, 0, 512);
	file = open(file_path.c_str(), O_RDONLY);
	count = read(file, buff, 512);
	close(file);

	origin = buff;

	// Get current configuration
	cmd = tool + grep_id;
	current = exec(cmd.c_str());
	ROS_DEBUG("Current configuration:\n%s", current.c_str());

	// Check if every device from current configuration is in the original
	int first = 0;
	std::string line;
	for (int i = 0; i < current.length(); i++) {
		if (current[i] == '\n') {
			line = current.substr(first, i - first);
			if (origin.find(line) == std::string::npos) {
				ROS_DEBUG("Device %s is not found in original configuration", line.c_str());
				diff += line + "\n";
				}
			first = i + 1;
			}
			
		}
	if (diff != "")
		diff.erase(diff.end() - 1);

	return diff;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// This function is used ro get the first word of the string, ignoring "sudo"
//
// str - input string
// returns string with the first word of the str, ignoring "sudo"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string first_word(std::string str) {
	std::string word = str;
	int begin = 0;
	for (int i=begin; i<str.length(); i++) {
		if (str[i] == ' ') {
			word = str.substr(begin, i - begin);
			if (word == "sudo") {
				begin = i + 1;
				continue;
				}				
			break;
			}
		}
	ROS_DEBUG("String: %s\nFirst word ignoring sudo: %s", str.c_str(), word.c_str());
	return word;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Execution of linux terminal command, returns output of this command
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string exec(const char* cmd) {
    std::tr1::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}
