/**
 * @file   base_id.hpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Sat Jan 10 23:37:38 2015
 * 
 * @brief  base type for tracker ids
 * 
 * 
 */

#ifndef BASE_ID_HPP
#define BASE_ID_HPP

template<typename Tracker = void>
struct BaseId 
{
  BaseId() : _id(-1) { }
  BaseId(int32_t id) : _id(id) { }

  inline int32_t& operator() () { return _id; }
  inline bool is_valid() { return _id != -1; }

  friend std::ostream& operator<< (std::ostream& os, const BaseId<Tracker>& id) {
    return os << Tracker::type_id <<":" << id._id;
  }

  int32_t const _id;
};


template<>
struct BaseId<void>
{

  BaseId() : _id(-1) { }
  BaseId(int64_t id) : _id(id) { }
  BaseId(int32_t tracker_id, int32_t trajectory_id) : id({tracker_id,trajectory_id}) {}
    //id.tracker(tracker_id), id.trajctory(trajectory_id) {}

  inline int64_t& operator() () { return _id; }
  inline bool is_valid() { return _id != -1; }

  friend std::ostream& operator<< (std::ostream& os, const BaseId<>& id) {
    return os << id.id.tracker <<":" << id.id.trajectory;
  }

  union {
    struct {
      int32_t tracker;
      int32_t trajectory;
    } id;
    int64_t _id;
  };
};

namespace std
{
  template<typename T>
  struct hash<BaseId<T> >
  {
    std::size_t operator() (const BaseId<T>& key) const
    {
      return std::hash<int>()(key._id);
    }
  };

  template<>
  struct hash<BaseId<> >
  {
    std::size_t operator() (const BaseId<>& key) const
    {
      return std::hash<int64_t>()(key._id);
    }
  };
}

template<typename T>
inline const bool operator==  (const BaseId<T>& lhs, const BaseId<T>& rhs) {
  return lhs._id == rhs._id;
}

template<typename T1, typename T2>
inline const bool operator==  (const BaseId<T1>& lhs, const BaseId<T2>& rhs) {
  return false;
}

template<typename T>
inline const bool operator!=  (const BaseId<T>& lhs, const BaseId<T>& rhs) {
  return !operator== (lhs._id,rhs._id);
}

template<typename T1, typename T2>
inline const bool operator!=  (const BaseId<T1>& lhs, const BaseId<T2>& rhs) {
  return true;
}

// maps id type to trajectory type, needs to be specialized by each tracker
template<typename T, typename = void>
struct id_traits {};


template<typename T>
inline BaseId<> to_runtime_id(T& id) {
  return BaseId<>(id_traits<T>::trajectory::type_id, id._id );
}

template<>
inline BaseId<> to_runtime_id(BaseId<>& id) {
  return id;
}

#endif
