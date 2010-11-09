#include "SSLVisionPacket.h"

SSLVisionPacket::SSLVisionPacket()
{
	return;
}

SSLVisionPacket::~SSLVisionPacket()
{
}

std::ostream& operator<< (std::ostream& output, const SSLVisionPacket& p_packet)
{
    int numRobots = p_packet.robots.size();
    output << numRobots << " ";
    for (int i = 0; i < numRobots; i++)
    {
        output << p_packet.robots[i].id << " ";
        output << p_packet.robots[i].location.x << " ";
        output << p_packet.robots[i].location.y << " ";
        output << p_packet.robots[i].heading << " ";
    }

    int numBalls = p_packet.balls.size();
    output << numBalls << " ";
    for (int i = 0; i < numBalls; i++)
    {
        output << p_packet.balls[i].x << " ";
        output << p_packet.balls[i].y << " ";
    }
    return output;
}

std::istream& operator>> (std::istream& input, SSLVisionPacket& p_packet)
{
    int numRobots, numBalls;
    input >> numRobots;
    p_packet.robots.resize(numRobots);
    for (int i = 0; i < numRobots; i++)
    {
        input >> p_packet.robots[i].id;
        input >> p_packet.robots[i].location.x;
        input >> p_packet.robots[i].location.y;
        input >> p_packet.robots[i].heading;
    }
    input >> numBalls;
    p_packet.balls.resize(numBalls);
    for (int i = 0; i < numBalls; i++)
    {
        input >> p_packet.balls[i].x;
        input >> p_packet.balls[i].y;
    }
    return input;
}

