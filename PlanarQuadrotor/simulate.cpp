/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <vector>
#include <thread>
#include <matplot/matplot.h>
#include <iostream>
#include <cmath>

void plot(float, PlanarQuadrotor);



const int SAMPLE_RATE = 1000; // Samples per second
const int AMPLITUDE = 10000;   // Amplitude of the wave
const float DURATION = 1;        // Duration in seconds


void generateSineWave(double frequency, int Amp, std::vector<int16_t>& samples, float duration) {
    int sampleCount = SAMPLE_RATE * duration;
    samples.resize(sampleCount);
    for (int i = 0; i < sampleCount; ++i) {
        double time = (double)i / SAMPLE_RATE;
        samples[i] = static_cast<int16_t>(Amp * sin(2.0 * M_PI * frequency * time));
    }
}

void audioCallback(void* userdata, Uint8* stream, int len) {
    std::vector<int16_t>* samples = (std::vector<int16_t>*)userdata;
    int16_t* buffer = (int16_t*)stream;
    int sampleCount = len / 2; 

    for (int i = 0; i < sampleCount; ++i) {
        buffer[i] = (*samples)[i];
    }
}


Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state<<SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);
    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */

    double frequency = 440.0; // częstotliwość tonu A
    std::vector<int16_t> samples;
    generateSineWave(frequency, AMPLITUDE, samples, DURATION);
    
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
    std::cerr << "Couldn't initialize SDL: " << SDL_GetError() << std::endl;
    }
    SDL_AudioSpec spec;
    spec.freq = SAMPLE_RATE;
    spec.format = AUDIO_S16SYS;
    spec.channels = 1;
    spec.samples = 100;
    spec.callback = audioCallback;
    spec.userdata = (void*)&samples;
    
    if (SDL_OpenAudio(&spec, NULL) < 0) {
    std::cerr << "Couldn't open audio: " << SDL_GetError() << std::endl;
    }
    SDL_PauseAudio(0);
    
    float time=0;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            Eigen::Vector2f AmpOffset = quadrotor.GravityCompInput() - K * quadrotor.GetControlState();
            generateSineWave(frequency, 1000 * (AmpOffset[0] + AmpOffset[1]), samples, DURATION);
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    //std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    goal_state<< x, y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p)
                {
                    std::thread wykres(plot, time, quadrotor);
                    wykres.detach();
                    //plot(time, quadrotor);
                }
                
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            time+=dt;
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}

void plot(float time, PlanarQuadrotor rotor)
{
    matplot::plot(rotor.y_ret(),rotor.x_ret());
    matplot::title("Y po X");
    matplot::show();
}
