/* particle_filter.cpp */

#include "particle_filter/particle_filter.h"

namespace pf
{
/******************************/
/*        Constructor         */
/******************************/

Particles::Particles()
{
  // Initialize the Subscribers
  _scanListener = _nh.subscribe("/scan", 10, &Particles::scanCallback, this);
  _odomListener = _nh.subscribe("/odom", 10, &Particles::odomCallback, this);
  ROS_INFO("HEY\n");
  // Initialize the Publishers
  _particlePublisher = _nh.advertise<geometry_msgs::PoseArray>("/particlecloud", 50);
  _posePublisher = _nh.advertise<geometry_msgs::Pose>("/amcl/pose", 50);
  // Crashes here, since pf in not initialized yet
  ROS_INFO("Particle filter created with %d particles\n", _pf->numParticles());

  /*
    * After that you can use the particle filter this way:
   * @code
   *   int numOfParticles = 500;
   *   MyMovementModel mm;              // create movement strategy
   *   MyObservationModel om;           // create observation strategy
   *   ParticleFilter<MyState> pf(numOfParticles, &om, &mm); // create filter
   *
   *   // run the filter loop
   *   bool doFilter = true;
   *   while (doFilter) {
   *     // update your observation model here
   *     // ...
   *     // update your movement model here (if necessary)
   *     // ...
   *
   *     // run one filter step, the filter uses the
   *     // observation model and the movement model
   *     // that were given in the constructor
   *     pf->filter();
   *
   *     // retrieve the best state and
   *     // do something with the result
   *     std::cout << pf->getBestState().getVariable1() << std::endl;
   *
   *   }
  */
  /*
      // getters of models
      libPF::ObservationModel<DroneState>* currentObservationModel = _pf.getObservationModel();
  // std::cout << (currentObservationModel == &_om) << std::endl;  // should be "1"
  libPF::MovementModel<DroneState>* currentMovementModel = _pf.getMovementModel();
  // std::cout << (currentMovementModel == &_mm) << std::endl;  // should be "1"

  // setters of models
  DroneObservationModel om2;
  DroneMovementModel mm2;
  _pf.setObservationModel(&om2);
  _pf.setMovementModel(&mm2);

  // change the resampling mode
  if (_pf.getResamplingMode() == libPF::RESAMPLE_NEFF)
  {
    _pf.setResamplingMode(libPF::RESAMPLE_ALWAYS);
  }

  // getter for resampling strategy
  libPF::ResamplingStrategy<DroneState>* resamplingStrategy = _pf.getResamplingStrategy();

  // setter for resampling strategy (not implemented here)
  // MyResamplingStrategy myResamplingStrategy;
  // pf.setResamplingStrategy(&myResamplingStrategy);

  // to integrate a known prior state use
  _pf.setPriorState(1.4);

  // or use a distribution to draw from
  DroneStateDistribution distribution;
  _pf.drawAllFromDistribution(distribution);

  for (int i = 0; i < 10000; i++)
  {
    _pf.filter();
    float best = _pf.getBestState();
    float mmse = _pf.getMmseEstimate();
    float best5percent = _pf.getBestXPercentEstimate(5.0);
  }

  // to reset the timer call
  _pf.resetTimer();

  // you can call the filter steps by hand
  double dt = 0.001;
  _pf.resample();
  _pf.drift(dt);
  _pf.diffuse(dt);
  _pf.measure();

  // iteration over the particle list
  libPF::ParticleFilter<DroneState>::ConstParticleIterator iter;
  unsigned int count = 0;
  for (iter = _pf.particleListBegin(); iter != _pf.particleListEnd(); ++iter)
  {
    libPF::Particle<DroneState>* particle = *iter;
    //    std::cout << "Particle [" << count++ << "]: weight = " << particle->getWeight()
    // << ", value = " << particle->getState() << std::endl;
  }

  // you can iterate over the list with indices as well and request weight and state from the particle filter
  for (unsigned int index = 0; index < _pf.numParticles(); index += 100)
  {
    //    std::cout << "Particle [" << index << "]: weight = " << _pf.getWeight(index) << ", value = " <<
    //    _pf.getState(index)
    //            << std::endl;
  }

  // number of effective particles
  // std::cout << "Number of effective particles: " << _pf.getNumEffectiveParticles() << std::endl;
  */
}

/******************************/
/*         Destructor         */
/******************************/

Particles::~Particles()
{
  ROS_INFO("Particles object destroyed");
}

/******************************/
/*         initState          */
/******************************/

void Particles::initState()
{
  double x_pos, y_pos, z_pos, roll, pitch, yaw;

  // _nh.param<std::string>("default_param", default_param, "default_value");
  // Pose parameters
  _nh.param<double>("/x_pos", x_pos, 0);
  _nh.param<double>("/y_pos", y_pos, 0);
  _nh.param<double>("/z_pos", z_pos, 0);

  _nh.param<double>("/roll", roll, 0);
  _nh.param<double>("/pitch", pitch, 0);
  _nh.param<double>("/yaw", yaw, 0);

  // Initialize state variables
  _ds.setXPos(x_pos);
  _ds.setYPos(y_pos);
  _ds.setZPos(z_pos);

  _ds.setRoll(roll);
  _ds.setRoll(pitch);
  _ds.setYaw(yaw);

  // Initialize particle filter's state
  _pf->setPriorState(_ds);
}

/******************************/
/*       scanCallback         */
/******************************/

void Particles::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
}

/******************************/
/*       odomCallback         */
/******************************/

void Particles::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
}

}  // namespace pf
