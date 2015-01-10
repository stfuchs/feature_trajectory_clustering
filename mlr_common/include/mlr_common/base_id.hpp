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

template<typename Tracker>
struct BaseId 
{
  BaseId() : _id(-1) { }
  BaseId(int id) : _id(id) { }

  inline int& operator() () { return _id; }
  inline bool is_valid() { return _id != -1; }

  friend std::ostream& operator<< (std::ostream& os, const BaseId<Tracker>& id) {
    return os << Tracker::type_id <<":" << id._id;
  }

  int const _id;
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

#endif
