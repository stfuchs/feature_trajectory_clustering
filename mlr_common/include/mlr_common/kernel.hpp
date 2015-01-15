/**
 * @file   kernel.hpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Sun Jan 11 00:24:47 2015
 * 
 * @brief  kernel template class
 * 
 * 
 */

#ifndef KERNEL_HPP
#define KERNEL_HPP

#include "mlr_common/trajectory.hpp"
#include "mlr_common/multi_type.hpp"

template<typename T1>
struct DistanceCalculator
{
  typedef typename T1::TimeT TimeT11;
  typedef typename T1::StateT StateT11;

  T1 const& fi;
  TimeT11 dt_inv;
  
  DistanceCalculator(T1 const& fi_) : fi(fi_) {
    dt_inv = 1./(fi.t[0] - fi.t[1]);
  }

  template<typename T2, typename D>
  void operator() (T2 const& fj, D& distances) const
  {
    typedef typename T1::type BT1;
    typedef typename T2::type BT2;
    typedef typename trajectory_type_promotion<BT1,BT2>::res_time_type TimeT12;
    typedef typename trajectory_type_promotion<BT1,BT2>::res_distance_type DistanceT12;
    if(fj.id==fi.id) return;

    unsigned int i = 0;
    while(fj.t[i] >= fi.t[1] && i<fj.t.size())
    {
      TimeT12 a = (fj.t[i] - fi.t[1]) * dt_inv;
      StateT11 xi = trajectory_policy<BT1,BT1>::lin_inter(fi.x[0],fi.x[1],a);
      DistanceT12 d = trajectory_policy<BT2,BT1>::distance(fj.x[i], xi);

      get(get(distances,fj.id)[i],fi.id) = d;
      ++i;
    }
  }
};

struct MatrixCalculator
{
  template<typename T1, typename T2, typename D>
  void operator() (T1 const& outer, T2 const& inner, D const& distances,
                   std::vector<float>& result)
  {
    typedef typename trajectory_type_promotion<typename T1::type, typename T2::type>::
      res_distance_type DistanceT;
    
    int n = 0;
    DistanceT sum = 0;
    DistanceT sum_sqr = 0;
    auto& que_outer = find(distances,outer.id);
    for(unsigned int i=0; i<que_outer.size(); ++i) // iterate outer distances
    {
      if (outer.t[i] >= inner.t.front()) continue;
      if (outer.t[i] < inner.t.back()) break;
      DistanceT const d = find(que_outer[i],inner.id);
      ++n;
      sum += d;
      sum_sqr += d*d;
    }
    auto& que_inner = find(distances,inner.id);
    for(unsigned int i=0; i<que_inner.size(); ++i) // iterate inner distances
    {
      if (inner.t[i] >= outer.t.front()) continue;
      if (inner.t[i] < outer.t.back()) break;
      DistanceT const d = find(que_inner[i],outer.id);
      ++n;
      sum += d;
      sum_sqr += d*d;
    }
    if(n!=0) {
      float n_inv = 1./n;
      result.push_back( exp(-(sum_sqr - sum*sum*n_inv)*n_inv) );
    }
    else {
      result.push_back( .5 );
    }
  }
};

struct PrintIds
{
  template<typename T>
  void operator() (T const& t) { std::cout << t.id << std::endl; }
};

template<typename OT>
struct CopyIds
{
  template<typename T>
  void operator() (T const& t, OT& vec_out) {
    cast_value<typename T::IdT>(vec_out).push_back(t.id);
  }
};

template<>
struct CopyIds<std::vector<int64_t> >
{
  template<typename T>
  void operator() (T const& t, std::vector<int64_t>& vec_out) {
    vec_out.push_back(to_runtime_id(t.id)._id);
  }
};




template<typename... Ts>
struct Kernel
{
  typedef type_array<Ts...> traj_types;
  typedef type_array<typename Ts::IdT...> id_types;

  template<typename T>
  struct dist_types
  {
    typedef type_array< 
      typename trajectory_type_promotion<typename T::type,typename Ts::type>::res_distance_type...
      > types;
    typedef MultiTypeDuo<std::unordered_map,id_types,types> distance_set;
    typedef std::deque<distance_set> distance_que;
  };

/*  // seems not to be working with older compilers:
  using dist_types = type_array< 
    typename trajectory_type_promotion<typename T::type,typename Ts::type>::res_distance_type... >;

  template<typename T>
  using DistanceSet = 

  template<typename T>
  using DistanceQue = std::deque<DistanceSet<T> >;
*/

  MultiTypeDuo<std::unordered_map,
               id_types,
               traj_types> data;

  MultiTypeDuo<std::unordered_map,
               id_types,
               type_array<typename dist_types<Ts>::distance_que...> > distances;

  template<typename IdT>
  inline bool isNew(IdT const& id) {
    return cast_key<IdT>(data).find(id) == cast_key<IdT>(data).end();
  }

  template<typename IdT>
  void newTrajectory(IdT const& id,
                     typename id_traits<IdT>::trajectory::StateT const& x_new,
                     typename id_traits<IdT>::trajectory::TimeT const& t_new)
  {
    typedef typename id_traits<IdT>::trajectory T;
    T& f = set(data,id);
    f.x.push_front(x_new);
    f.t.push_front(t_new);
    typename dist_types<T>::distance_que& d = 
      set(distances,id, typename dist_types<T>::distance_que());
    d.push_front(typename dist_types<T>::distance_set()); // push empty distance_set onto deque
  }

  template<typename IdT>
  void updateTrajectory(IdT const& id,
                        typename id_traits<IdT>::trajectory::StateT const& x_new,
                        typename id_traits<IdT>::trajectory::TimeT const& t_new)
  {
    typedef typename id_traits<IdT>::trajectory T;
    T& f = find(data,id);
    f.x.push_front(x_new);
    f.t.push_front(t_new);
    typename dist_types<T>::distance_que& d = find(distances,id);
    d.push_front(typename dist_types<T>::distance_set()); // push empty distance_set onto deque
    while( (t_new - f.t.back() > T::timespan || f.t.size() > T::n_max) && f.t.size() > T::n_min)
    {
      f.t.pop_back();
      f.x.pop_back();
      d.pop_back();
    }
    // update distances of all other trajectories to the new time interval (t,t-1)
    // of this trajectory:
    foreach_value(data,DistanceCalculator<T>(f),distances);
  }

  void computeKernelMatrix(Eigen::MatrixXf& K, MultiType<std::vector,id_types>& ids)
  {
    std::vector<float> result;
    foreach_twice(data,MatrixCalculator(),distances,result);
    int n = .5+.5*sqrt(1.+8.*result.size());
    K = Eigen::MatrixXf::Ones(n,n);
    typename std::vector<float>::iterator it = result.begin();
    for(int i=0;i<n;++i) for(int j=i+1;j<n; ++j) K(i,j) = K(j,i) = *it++;


    foreach_value(data, CopyIds<MultiType<std::vector,id_types> >(), ids);
  }

  void computeKernelMatrixData(std::vector<float>& v_data, std::vector<int64_t>& v_ids)
  {
    foreach_twice(data, MatrixCalculator(), distances, v_data);
    foreach_value(data, CopyIds<std::vector<int64_t> >(), v_ids);
  }
};

#endif
