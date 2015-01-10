#ifndef MULTI_TYPE_HPP
#define MULTI_TYPE_HPP

#include <type_traits>

template<typename... Ts>
struct type_array
{
  typedef type_array<> next;
  typedef void sub_type;
  enum { N = 0 };
};

template<typename T, typename... Ts>
struct type_array<T,Ts...> : type_array<Ts...>
{
  typedef type_array<Ts...> next;
  typedef T sub_type;
  enum { N = next::N+1 };
};

template<int N, typename T, typename... Ts>
struct repeat : repeat<N-1, T, T, Ts...> { };

template<typename T, typename... Ts>
struct repeat<0, T, Ts...>
{
  typedef type_array<Ts...> result;
};


template<template<typename,typename...> class Subtype, typename Values>
struct MultiType : MultiType<Subtype, typename Values::next>
{
  typedef MultiType<Subtype, typename Values::next> next;
  typedef typename Values::sub_type value_type;
  typedef Subtype<value_type> sub_type;
  enum { N = next::N+1 };
  sub_type _M_impl;
};

template<template<typename,typename...> class Subtype>
struct MultiType<Subtype,type_array<> > { enum{ N = 0 }; };


template<template<typename,typename...> class Subtype, typename Keys, typename Values>
struct MultiTypeDuo : MultiTypeDuo<Subtype, typename Keys::next, typename Values::next>
{
  typedef MultiTypeDuo<Subtype, typename Keys::next, typename Values::next> next;
  typedef typename Keys::sub_type key_type;
  typedef typename Values::sub_type value_type;
  typedef Subtype<key_type,value_type> sub_type;
  enum { N = next::N+1 };
  sub_type _M_impl;
};

template<template<typename,typename...> class Subtype>
struct MultiTypeDuo<Subtype,type_array<>,type_array<> > { enum{ N = 0 }; };


/***************************************************************************************
 *  Type Functions
 **************************************************************************************/

template<typename T, typename query, typename = void>
struct by_value_type
{
  typedef typename by_value_type<typename T::next, query>::type type;
};

template<typename T, typename query>
struct by_value_type<T,query,typename std::enable_if<
                               std::is_same<query,typename T::value_type>::value ||
                               std::is_same<query,typename T::value_type const>::value>::type>
{
  typedef T type;
};


template<typename T, typename query, typename = void>
struct by_key_type
{
  typedef typename by_key_type<typename T::next, query>::type type;
};

template<typename T, typename query>
struct by_key_type<T,query,typename std::enable_if<
                             std::is_same<query,typename T::key_type>::value ||
                             std::is_same<query,typename T::key_type const>::value>::type>
{
  typedef T type;
};


template<typename T, int I, typename = void>
struct by_index
{
  typedef typename by_index<typename T::next, I-1>::type type;
};

template<typename T, int I>
struct by_index<T, I, typename std::enable_if<I==0>::type >
{
  typedef T type;
};


/***************************************************************************************
 *  Getter
 **************************************************************************************/

template<typename value_type,typename T>
inline typename by_value_type<T,value_type>::type::sub_type& cast_value(T& multi) {
  return static_cast<typename by_value_type<T,value_type>::type &>(multi)._M_impl;
}

template<typename value_type,typename T>
inline typename by_value_type<T,value_type>::type::sub_type const& cast_value(T const& multi) {
  return static_cast<typename by_value_type<T,value_type>::type const &>(multi)._M_impl;
}


template<typename key_type,typename T>
inline typename by_key_type<T,key_type>::type::sub_type& cast_key(T& multi) {
  return static_cast<typename by_key_type<T,key_type>::type &>(multi)._M_impl;
}

template<typename key_type,typename T>
inline typename by_key_type<T,key_type>::type::sub_type const& cast_key(T const& multi) {
  return static_cast<typename by_key_type<T,key_type>::type const &>(multi)._M_impl;
}


template<int I, typename T>
inline typename by_index<T,I>::type::sub_type& cast_index(T& multi) {
  return static_cast<typename by_index<T,I>::type &>(multi)._M_impl;
}

template<int I, typename T>
inline typename by_index<T,I>::type::sub_type const& cast_index(T const& multi) {
  return static_cast<typename by_index<T,I>::type const&>(multi)._M_impl;
}


template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& get(T& m, key_type& key) {
  return cast_key<key_type>(m)[key];
}

template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& get(T & m, key_type const& key) {
  return cast_key<const key_type>(m)[key];
}

template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type const& get(T const& m, key_type const& key){
  return cast_key<const key_type>(m)[key];
}


template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& find(T& m, key_type& key) {
  return cast_key<key_type>(m).find(key)->second;
}

template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& find(T & m, key_type const& key) {
  return cast_key<const key_type>(m).find(key)->second;
}

template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type const& find(T const& m, key_type const& key){
  return cast_key<const key_type>(m).find(key)->second;
}

template<typename T, typename key_type, typename value_type>
bool find(T& m, key_type& key, value_type& value)
{
  auto res = cast_key<key_type>(m).find(key);
  if (res == cast_key<key_type>(m).end()) return false;
  value = res->second;
  return true;
}

template<typename T, typename key_type, typename value_type>
bool find(T& m, key_type const& key, value_type& value)
{
  auto res = cast_key<const key_type>(m).find(key);
  if (res == cast_key<const key_type>(m).end()) return false;
  value = res->second;
  return true;
}

template<typename T, typename key_type, typename value_type>
bool find(T const& m, key_type const& key, value_type& value)
{
  auto res = cast_key<const key_type>(m).find(key);
  if (res == cast_key<const key_type>(m).end()) return false;
  value = res->second;
  return true;
}


template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& set(T& m, key_type & key)
{
  typedef typename by_key_type<T,key_type>::type::value_type V;
  return cast_key<key_type>(m).insert(std::pair<key_type,V>(key,V(key))).first->second;
}

template<typename T, typename key_type>
inline typename by_key_type<T,key_type>::type::value_type& set(T& m, key_type const& key)
{
  typedef typename by_key_type<T,key_type>::type::value_type V;
  return cast_key<const key_type>(m).insert(std::pair<key_type,V>(key,V(key))).first->second;
}

template<typename T, typename key_type, typename value_type>
inline value_type& set(T& m, key_type & key, value_type& value)
{
  return cast_key<key_type>(m).insert(std::make_pair(key,value)).first->second;
}

template<typename T, typename key_type, typename value_type>
inline value_type& set(T& m, key_type const& key, value_type const& value)
{
  return cast_key<key_type>(m).insert(std::make_pair(key,value)).first->second;
}



/***************************************************************************************
 *  loop
 **************************************************************************************/


template<typename T, typename = void>
struct for_each_type
{
  template<typename FuncT, typename... ArgsT>
  inline void operator() (T& m, FuncT f, ArgsT&... args)
  {
    f(m, args...);
    for_each_type<typename T::next>()(m,f,args...);
  }
};

template<typename T>
struct for_each_type<T, typename std::enable_if<T::N == 1>::type>
{
  template<typename FuncT, typename... ArgsT>
  inline void operator() (T& m, FuncT f, ArgsT&... args)
  {
    f(m, args...);
  }
};


struct for_each_element
{
  template<typename T, typename FuncT, typename... ArgsT>
  inline void operator() (T& m, FuncT f, ArgsT& ... args)
  {
    typename T::sub_type::iterator it = m._M_impl.begin();
    for(; it != m._M_impl.end(); ++it)
    {
      f(it, args...);
    }
  }
};


template<typename T=void, typename IterT=void>
struct for_each_element_inner
{
  template<typename FuncT, typename... ArgsT>
  inline void operator() (T& m, IterT& it_outer, FuncT f, ArgsT& ... args)
  {
    typename T::sub_type::iterator it_inner = m._M_impl.begin();
    for(; it_inner!= m._M_impl.end(); ++it_inner)
    {
      f(it_outer->second,it_inner->second,args...);
    }
  }
};

template<typename T>
struct for_each_element_inner<T,typename T::sub_type::iterator>
{
  typedef typename T::sub_type::iterator IterT;

  template<typename FuncT, typename... ArgsT>
  inline void operator() (T& m, IterT& it_outer, FuncT f, ArgsT& ... args)
  {
    IterT it_inner = it_outer; ++it_inner;
    for(; it_inner!= m._M_impl.end(); ++it_inner)
    {
      f(it_outer->second,it_inner->second,args...);
    }
  }
};

template<>
struct for_each_element_inner<void,void>
{
  template<typename T, typename IterT, typename FuncT, typename... ArgsT>
  inline void operator() (T& m, IterT& it_outer, FuncT f, ArgsT& ... args) {
    for_each_element_inner<T,IterT>()(m,it_outer,f,args...);
  }
};


struct for_each_element_outer
{
  template<typename T, typename FuncT, typename... ArgsT>
  inline void operator() (T& m, FuncT f, ArgsT& ... args)
  {
    typedef typename T::sub_type::iterator IterT;
    for(IterT it = m._M_impl.begin(); it != m._M_impl.end(); ++it)
    {
      for_each_type<T>()(m,for_each_element_inner<>(),it,f,args...);
      //for_each_element_inner<T,IterT>()(m,it,f,args...);
    }
  }
};


struct for_each_value_element
{
  template<typename T, typename FuncT, typename... ArgsT>
  inline void operator() (T& m, FuncT f, ArgsT& ... args)
  {
    typename T::sub_type::iterator it = m._M_impl.begin();
    for(; it != m._M_impl.end(); ++it)
    {
      f(it->second, args...);
    }
  }
};

template<typename T, typename FuncT, typename... ArgsT>
inline void foreach(T& m, FuncT f, ArgsT& ... args)
{
  for_each_type<T>()(m, for_each_element(), f, args...);
}

template<typename T, typename FuncT, typename... ArgsT>
inline void foreach_value(T& m, FuncT f, ArgsT& ... args)
{
  for_each_type<T>()(m, for_each_value_element(), f, args...);
}

template<typename T, typename FuncT, typename... ArgsT>
inline void foreach_twice(T& m, FuncT f, ArgsT& ... args)
{
  for_each_type<T>()(m, for_each_element_outer(), f, args...);
}

#endif
