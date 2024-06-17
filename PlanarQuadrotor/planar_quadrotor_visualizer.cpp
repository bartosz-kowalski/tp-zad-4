#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float screen_x, screen_y, q_theta;

    /* x, y, theta coordinates */
    screen_x = state[0];
    screen_y = state[1];
    q_theta = state[2];

    //draw goal
    Eigen::RowVectorXf controlstate = quadrotor_ptr->GetControlState();

    int goalx = static_cast<int>((state[0] - controlstate[0]));
    int goaly = static_cast<int>((state[1] - controlstate[1]));

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), goalx, goaly, 3, 0xFF0000FF);


    // Set quadrotor body color
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);

    // quadrator body
    int body_width = 60;
    int body_height = 10;

    int xbody = screen_x - cos(-q_theta) * (body_width / 2);
    int xbody1 = screen_x + cos(-q_theta) * (body_width / 2);
    int ybody = screen_y - sin(-q_theta) * (body_width / 2);
    int ybody1 = screen_y + sin(-q_theta) * (body_width / 2);
    SDL_RenderDrawLine(gRenderer.get(), xbody, ybody, xbody1, ybody1);

    int xoffset = body_height * cos(1.57079633 + q_theta);
    int yoffset = body_height * sin(1.57079633 + q_theta);

    int x0body1 = xbody + xoffset;
    int y0body1 = ybody - yoffset;

    int x1body1 = xbody1 + xoffset;
    int y1body1 = ybody1 - yoffset;

    SDL_RenderDrawLine(gRenderer.get(), xbody, ybody, x0body1, y0body1);
    SDL_RenderDrawLine(gRenderer.get(), xbody1, ybody1, x1body1, y1body1);
    SDL_RenderDrawLine(gRenderer.get(), x0body1, y0body1, x1body1, y1body1);

    // propeller positions
    int propeller_length = 20;
    int propeller_offset = 10;
    int propeller1_x = x0body1 + (propeller_offset)*cos(1.57079633 + q_theta);
    int propeller1_y = y0body1 - (propeller_offset)*sin(1.57079633 + q_theta);

    int propeller1_x1 = propeller1_x + cos(-q_theta) * propeller_length;
    int propeller1_y1 = propeller1_y + sin(-q_theta) * propeller_length;

    int propeller2_x = x1body1 + (propeller_offset)*cos(1.57079633 + q_theta);
    int propeller2_y = y1body1 - (propeller_offset)*sin(1.57079633 + q_theta);

    int propeller2_x1 = propeller2_x - cos(-q_theta) * propeller_length;
    int propeller2_y1 = propeller2_y - sin(-q_theta) * propeller_length;

    // Draw propellers
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0xFF, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), propeller1_x, propeller1_y, propeller1_x1, propeller1_y1);
    SDL_RenderDrawLine(gRenderer.get(), propeller2_x, propeller2_y, propeller2_x1, propeller2_y1);

    //// propellers animiation

    int propeller1_x1pos = propeller1_x + cos(-q_theta) * (propeller_length / 2);
    int propeller1_y1pos = propeller1_y + sin(-q_theta) * (propeller_length / 2);
    int propeller2_x1pos = propeller2_x - cos(-q_theta) * (propeller_length / 2);
    int propeller2_y1pos = propeller2_y - sin(-q_theta) * (propeller_length / 2);

    static float propeller_angle = 0.0;
    propeller_angle += 0.1;
    int propeller_radius = 10;

    for (int i = 0; i < 2; ++i) {
        float angle = propeller_angle + i * M_PI;
        int prop_x = propeller1_x1pos + propeller_radius * cos(angle);
        int prop_y = propeller1_y1pos + propeller_radius * sin(angle);
        SDL_RenderDrawLine(gRenderer.get(), propeller1_x1pos, propeller1_y1pos, prop_x, prop_y);

        prop_x = propeller2_x1pos + propeller_radius * cos(angle);
        prop_y = propeller2_y1pos + propeller_radius * sin(angle);
        SDL_RenderDrawLine(gRenderer.get(), propeller2_x1pos, propeller2_y1pos, prop_x, prop_y);
    }
}
