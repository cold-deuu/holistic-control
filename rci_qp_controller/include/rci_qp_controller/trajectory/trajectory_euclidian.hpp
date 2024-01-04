#ifndef __trajectory_euclidian_hpp__
#define __trajectory_euclidian_hpp__

#include "rci_qp_controller/trajectory/trajectory_base.hpp"
#include <vector>


namespace qp_controller{
    namespace trajectory{
        class TrajectoryEuclidianConstant : public TrajectoryBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                
                typedef math::Vector         Vector;
                typedef math::ConstRefVector ConstRefVector;

                TrajectoryEuclidianConstant(const std::string & name);

                TrajectoryEuclidianConstant(const std::string & name, ConstRefVector ref);

                unsigned int size() const;

                void setReference(ConstRefVector ref);

                const TrajectorySample & operator()(double time);

                const TrajectorySample & computeNext();

                void getLastSample(TrajectorySample & sample) const;

                bool has_trajectory_ended() const;

                const std::vector<Eigen::VectorXd> & getWholeTrajectory();

            protected:
                Vector m_ref;
                std::vector<Eigen::VectorXd> traj_;
        };

        class TrajectoryEuclidianCubic : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef math::Vector         Vector;
            typedef math::ConstRefVector ConstRefVector;

			TrajectoryEuclidianCubic(const std::string & name);
			TrajectoryEuclidianCubic(const std::string & name, ConstRefVector init_M, ConstRefVector goal_M, const double & duration, const double & stime);

			unsigned int size() const;
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;
			void setReference(ConstRefVector ref);
			void setInitSample(ConstRefVector init_M);
			void setGoalSample(ConstRefVector goal_M);
			void setDuration(const double & duration);
			void setCurrentTime(const double & time);
			void setStartTime(const double & time);
            const std::vector<Eigen::VectorXd> & getWholeTrajectory();

		protected:
			Vector m_ref;
			Vector m_init, m_goal, m_cubic;
			double m_duration, m_stime, m_time;
            std::vector<Eigen::VectorXd> traj_;
		};

        
    }
}

#endif 