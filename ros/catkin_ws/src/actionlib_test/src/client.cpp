#include <actionlib_test/DoDishesAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<actionlib_test::DoDishesAction> Client;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "do_dishes_client");

    Client client("do_dishes", true); // true -> don't need ros::spin()
    client.waitForServer();           // Waits for the ActionServer to connect to this client
    actionlib_test::DoDishesGoal goal;
    // Fill in goal here
    client.sendGoal(goal);                    // Sends a goal to the ActionServer
    client.waitForResult(ros::Duration(5.0)); // Blocks until this goal finishes
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Yay! The dishes are now clean\n");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}