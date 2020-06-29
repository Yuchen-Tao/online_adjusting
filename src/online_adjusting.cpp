#include "online_adjusting.h"

typedef std::chrono::high_resolution_clock ChronoTime;

Transform::Transform()
{
    // parameter initialization
    heading = 0;
    x = 0;
    y = 0;
    time_counter = 0;

    rotation_angle = 0;
    dx = 0;
    dy = 0;
    msg_pub_ = n_.advertise<automated_driving_msgs::ObjectStateArray>(
        "/transformed", 1);
    sub_ = n_.subscribe("/sensor/laser/objects", 100, &Transform::transformation, this);
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("SDL2 Keyboard/Mouse events", SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED, 200, 200, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
    keyboard_control();
}

void Transform::transformation(automated_driving_msgs::ObjectStateArray msg_array_in)
{
    msg_array.objects.clear();
    //reveive ObjectStateArray and perform transformation
    for (auto &msg_in : msg_array_in.objects)
    {
        msg = msg_in;

        msg.motion_state.header.frame_id = "map";
        msg.motion_state.child_frame_id = "vehicle";

        //rotation
        double s = sin(rotation_angle / 180 * M_PI);
        double c = cos(rotation_angle / 180 * M_PI);

        x = msg_in.motion_state.pose.pose.position.x * c - msg_in.motion_state.pose.pose.position.y * s;
        y = msg_in.motion_state.pose.pose.position.x * s + msg_in.motion_state.pose.pose.position.y * c;

        //translation
        x += dx;
        y += dy;

        //absolute speed
        v = sqrt(pow(msg.motion_state.twist.twist.linear.x,2)+pow(msg.motion_state.twist.twist.linear.y,2)+pow(msg.motion_state.twist.twist.linear.z,2));

        msg.motion_state.pose.pose.position.x = x;
        msg.motion_state.pose.pose.position.y = y;

        msg.motion_state.twist.twist.linear.x = v;
        msg.motion_state.twist.twist.linear.y = 0;
        msg.motion_state.twist.twist.linear.z = 0;

        msg_array.header.stamp = msg.header.stamp;
        msg_array.header.frame_id = "map";
        msg_array.objects.emplace_back(msg);
    }
    msg_pub_.publish(msg_array);
}

void Transform::keyboard_control()
{
    SDL_Event event;
    int quit = 0;
    ros::Rate loop_rate(50);

    double angle_to_rad = M_PI / 180;

    while (!quit)
    {

        ros::spinOnce();
        loop_rate.sleep();

        const Uint8 *state = SDL_GetKeyboardState(NULL);
        if (state[SDL_SCANCODE_UP])
        {
            rotation_angle += 0.05;
            ROS_INFO("Rotation: %f", rotation_angle);
        }
        if (state[SDL_SCANCODE_DOWN])
        {
            rotation_angle -= 0.05;
            ROS_INFO("Rotation: %f", rotation_angle);
        }
        if (state[SDL_SCANCODE_A])
        {
            dx += 0.02;
            ROS_INFO("X translation: %f", dx);
        }
        if (state[SDL_SCANCODE_D])
        {
            dx -= 0.02;
            ROS_INFO("X translation: %f", dx);
        }
        if (state[SDL_SCANCODE_S])
        {
            dy += 0.02;
            ROS_INFO("Y translation: %f", dy);
        }
        if (state[SDL_SCANCODE_W])
        {
            dy -= 0.02;
            ROS_INFO("Y translation: %f", dy);
        }

        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            // Look for a keypress
            case SDL_KEYDOWN:
                // Check the SDLKey values and move change the coords
                switch (event.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                    ROS_INFO("%s", "quit simulation");
                    quit = 1;
                    break;
                default:
                    break;
                }
                break;
            case SDL_QUIT:
                ROS_INFO("%s", "quit simulation");
                quit = 1;
                break;
            default:
                break;
            }
        }
    }
}
