

namespace modules{
namespace motion{

		class KickBalancer{
			private:
				bool running;

			public:
				void start(){
					running = true;
				}
				void stop();
				bool isRunning(){return running;}
				Transform3D getFootPose();
		};

		class FootLifter{
			private:
				float footHeight;
				bool running;
			public:
				void start(){
					running = true;
				}
				void stop();
				bool isRunning(){return running;}
				Transform3D getFootPose();
		};

		class Kicker{
			private:
				bool running;

			public:
				void kick(){
					running = true;
				}
				bool isRunning(){return running;}
				Transform3D getFootPose();	
		}:

	}
}