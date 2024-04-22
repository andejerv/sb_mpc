#include <SFML/Graphics.hpp>
#include "SB_MPC.h"

// Constants
constexpr int WINDOW_SIZE = 800;
constexpr int SCALE = 5;
constexpr int DELTA_T_MS = 100;

double find_angle(double x1, double y1, double x2, double y2){
    double angle = atan2(y2-y1, x2-x1);
    return angle;
}

void update_agent(BB_MODEL& agent, double hdg, double vel){
    Eigen::VectorXd state = agent.getState();
    state(0) += vel*SCALE*cos(hdg)*(static_cast<float>(DELTA_T_MS)/1000.0);
    state(1) += vel*SCALE*sin(hdg)*(static_cast<float>(DELTA_T_MS)/1000.0);

    agent.setState(state);
}

void update_agentShape(sf::CircleShape& agentShape, BB_MODEL& agent){
    Eigen::VectorXd state = agent.getState();
    agentShape.setPosition((WINDOW_SIZE/2)+(state(0)*SCALE), (WINDOW_SIZE/2)-((state(1)*SCALE)));
}

void update_dynamic_obstacles(std::vector<Object*> & obsta){
    // Update dynamic obstacles
    for(auto obs : obsta){
        if(obs->getType() == object_type::DYNAMIC){
            obs->setX(obs->getX() + obs->getVelocity()*cos(obs->getHeading())*(static_cast<float>(DELTA_T_MS)/1000.0)*SCALE);
            obs->setY(obs->getY() + obs->getVelocity()*sin(obs->getHeading())*(static_cast<float>(DELTA_T_MS)/1000.0)*SCALE);
        }
    }

}

std::vector<sf::CircleShape> make_shapes(std::vector<Object*> & obstacles){
    std::vector<sf::CircleShape> shapes;
    for (Object* obj : obstacles){
        sf::CircleShape circle(obj->getSize()*SCALE);
        circle.setOrigin(obj->getSize()*SCALE, obj->getSize()*SCALE);
        circle.setPosition((WINDOW_SIZE/2)+(obj->getX()*SCALE), (WINDOW_SIZE/2)-(obj->getY()*SCALE));

        circle.setFillColor(sf::Color(255, 0, 0));
        circle.setOutlineThickness((obj->getSafeDistance()-obj->getCollisionDistance())*SCALE);
        circle.setOutlineColor(sf::Color(250, 140, 100));

        shapes.push_back(circle);
    }
    return shapes;
}


int main(){

    Stopwatch sw;

    SB_MPC mpc(20*30, 0.05);
    BB_MODEL agent(0, -70);

    std::vector<Object*> obstacles;

    // Push back objects with different sizes and positions to obstacles
    /*obstacles.push_back(new Object(0, 0, 2, 2, 10));
    obstacles.push_back(new Object(20, 20, 3, 3, 10));
    obstacles.push_back(new Object(-20, 20, 3, 3, 10));*/
    obstacles.push_back(new Object(-5, 0, 3, 3, 10));
    obstacles.push_back(new Object(-10, 10, 3, 3, 10));
    obstacles.push_back(new Object(10, 0, 3, 3, 10));
    obstacles.push_back(new Object(40, -10, 3, 3, 15, 1, M_PI));
    obstacles.push_back(new Object(-35, 35, 3, 3, 15, 1, (3.5/2)*M_PI));

    // Make agent
    sf::CircleShape agentShape(20);
    agentShape.setOrigin(20, 20);
    agentShape.setPosition(WINDOW_SIZE/2, WINDOW_SIZE/2);
    agentShape.setFillColor(sf::Color::Black);

    // Make goal position
    sf::CircleShape goalPosition(10);
    goalPosition.setOrigin(10, 10);
    goalPosition.setPosition(400, 100);
    goalPosition.setFillColor(sf::Color(0, 255, 0));

    // Control variables
    double vel_os_best = 1;
    double hdg_os_best = 0;


    sf::RenderWindow window(sf::VideoMode(WINDOW_SIZE, WINDOW_SIZE), "SB-MPC Simulation");
    // Window loop
    while (window.isOpen())
    {
        // Event handling
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed){
                window.close();
            }

            if(event.type == sf::Event::KeyPressed){
                if(event.key.code == sf::Keyboard::Space){
                    // Hold simulation until pressed again
                    while(true){
                        window.pollEvent(event);
                        if (event.type == sf::Event::Closed){
                            window.close();
                            return 0;
                        }

                        if(event.type == sf::Event::KeyPressed){
                            if(event.key.code == sf::Keyboard::Space){
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Simulate agent
        double rad = find_angle((WINDOW_SIZE/2)+agentShape.getPosition().x, (WINDOW_SIZE/2)-agentShape.getPosition().y, (WINDOW_SIZE/2)+goalPosition.getPosition().x, (WINDOW_SIZE/2)-goalPosition.getPosition().y);
        sw.start();
        mpc.getBestControlOffset(hdg_os_best, vel_os_best, rad, 1, &agent, obstacles);
        double elapsed = sw.stop();
        std::cout << "Elapsed time: " << elapsed << " ms" << std::endl;
        update_agent(agent, rad+hdg_os_best, vel_os_best);
        update_agentShape(agentShape, agent);

        // Update dynamic obstacles
        update_dynamic_obstacles(obstacles);
        // Make shapes for every obstacle
        std::vector<sf::CircleShape> shapes = make_shapes(obstacles);
        

        // Reset window
        window.clear(sf::Color::Blue);

        // Draw obstacles
        for (sf::CircleShape shape : shapes){
            window.draw(shape);
        }

        // Draw goal position
        window.draw(goalPosition);

        // Draw agent
        window.draw(agentShape);


        // Update display
        window.display();
        sf::sleep(sf::milliseconds(DELTA_T_MS));
    }

    for( auto obstacle : obstacles){
        delete obstacle;
    }

    return 0;
}