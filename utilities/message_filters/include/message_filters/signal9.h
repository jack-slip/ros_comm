/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

 #ifndef MESSAGE_FILTERS_SIGNAL_H
 #define MESSAGE_FILTERS_SIGNAL_H
 
 #include <boost/noncopyable.hpp>
 #include "connection.h"
 #include "null_types.h"
 #include <ros/message_event.h>
 #include <ros/parameter_adapter.h>
 #include <boost/bind/bind.hpp>
 #include <boost/thread/mutex.hpp>
 #include <vector>
 #include <algorithm>
 #include <boost/shared_ptr.hpp>
 #include <boost/function.hpp>
 #include <tuple>
 #include <utility>
 #include <type_traits>
 #include <cstddef>
 
 namespace message_filters {
 
 // Helper function that “adapts” a message event by constructing a new event
 // with the copy flag set to (force_copy || event.nonConstWillCopy()).
 template<typename Event>
 Event adapt_event(const Event& e, bool force_copy) {
   return Event(e, force_copy || e.nonConstWillCopy());
 }
 
 // --------------------------------------------------------------------
 // Base Callback Helper (variadic)
 // This abstract base class defines the interface for callbacks. It is
 // templated on the message types that will be provided (one per signal).
 // (Each message event type is: ros::MessageEvent<const Msg>.)
 template<typename... Msgs>
 class CallbackHelper {
 public:
   virtual ~CallbackHelper() {}
   virtual void call(bool nonconst_force_copy,
                     const ros::MessageEvent<const Msgs>&... events) = 0;
   typedef boost::shared_ptr< CallbackHelper > Ptr;
 };
 
 // --------------------------------------------------------------------
 // CallbackHelperT
 // This helper wraps a callback (a boost::function) whose parameters
 // are the “adapted” types defined via ros::ParameterAdapter.
 // That is, a user supplies a callback with signature:
 //    void( Param0, Param1, …, ParamN )
 // and ParameterAdapter<Param_i>::Message is required to be the corresponding
 // message type (Msg_i) from the SignalN.
 template<typename... Params>
 class CallbackHelperT : public CallbackHelper<typename ParameterAdapter<Params>::Message...> {
 public:
   typedef boost::function<void(typename ParameterAdapter<Params>::Parameter...)> Callback;
 
   CallbackHelperT(const Callback& cb)
     : callback_(cb)
   { }
 
   virtual void call(bool nonconst_force_copy,
                     const ros::MessageEvent<const typename ParameterAdapter<Params>::Message>&... events)
   {
     // For each message event, adapt it (potentially forcing a copy)
     // then extract the callback parameter via ParameterAdapter.
     callback_( ParameterAdapter<Params>::getParameter(adapt_event(events, nonconst_force_copy))... );
   }
 
 private:
   Callback callback_;
 };
 
 // --------------------------------------------------------------------
 // Function Traits
 // These templates help deduce the number of arguments (arity) and argument
 // types for a callable (function pointer, member function, lambda, etc.).
 template<typename T>
 struct function_traits;
 
 // specialization for function pointers
 template<typename R, typename... Args>
 struct function_traits<R(*)(Args...)> {
   typedef R return_type;
   static constexpr std::size_t arity = sizeof...(Args);
   typedef std::tuple<Args...> args_tuple;
 };
 
 // specialization for member function pointers
 template<typename R, typename C, typename... Args>
 struct function_traits<R(C::*)(Args...)> {
   typedef R return_type;
   static constexpr std::size_t arity = sizeof...(Args);
   typedef std::tuple<Args...> args_tuple;
 };
 
 template<typename R, typename C, typename... Args>
 struct function_traits<R(C::*)(Args...) const> {
   typedef R return_type;
   static constexpr std::size_t arity = sizeof...(Args);
   typedef std::tuple<Args...> args_tuple;
 };
 
 // For functors and lambdas, use the operator() member function.
 template<typename F>
 struct function_traits : function_traits<decltype(&F::operator())> {};
 
 // --------------------------------------------------------------------
 // SignalN Class (variadic)
 // The SignalN class is now templated on an arbitrary number of message types.
 // Its call() method takes one message event per message type. The stored callbacks
 // are wrapped via CallbackHelperT so that a user’s callback is called with the
 // “adapted” parameter types (as defined by ros::ParameterAdapter).
 //
 // The primary addCallback() overload accepts a boost::function with signature:
 //    void( Parameter0, Parameter1, …, ParameterN )
 // where for each i, ParameterAdapter<Parameter_i>::Message must be the same as the
 // i‑th message type used to instantiate the SignalN.
 //
 // (For convenience, one can use boost::bind to “pad” a callback if fewer than N parameters
 // are desired.)
 template<typename... Msgs>
 class SignalN {
   typedef typename CallbackHelper<Msgs...>::Ptr CallbackHelperPtr;
   typedef std::vector<CallbackHelperPtr> CallbackHelperVector;
 public:
   // Primary addCallback overload. (Users who wish to supply a callback with fewer arguments
   // should adapt it via boost::bind.)
   template<typename... Params>
   Connection addCallback(const boost::function<void(typename ParameterAdapter<Params>::Parameter...)>& callback)
   {
     // (Optionally one could static_assert that for each index i, 
     //  ParameterAdapter<Params>::Message is the same as the i-th type in Msgs...)
     CallbackHelperT<Params...>* helper = new CallbackHelperT<Params...>(callback);
     boost::mutex::scoped_lock lock(mutex_);
     callbacks_.push_back(CallbackHelperPtr(helper));
     return Connection(boost::bind(&SignalN::removeCallback, this, callbacks_.back()));
   }
 
   // Overload for callable objects (function pointers, functors, lambdas)
   // whose number of arguments exactly matches the number of message types.
   template<typename F>
   typename std::enable_if<
     (function_traits<F>::arity == sizeof...(Msgs)),
     Connection
   >::type
   addCallback(F callback)
   {
     boost::function<void(Msgs...)> func = callback;
     return addCallback(func);
   }
 
   // Remove a callback.
   void removeCallback(const CallbackHelperPtr& helper)
   {
     boost::mutex::scoped_lock lock(mutex_);
     typename CallbackHelperVector::iterator it = std::find(callbacks_.begin(), callbacks_.end(), helper);
     if (it != callbacks_.end())
     {
       callbacks_.erase(it);
     }
   }
 
   // Call all callbacks with the given message events.
   // The message events are passed in the same order as the signal’s message types.
   void call(const ros::MessageEvent<const Msgs>&... events)
   {
     boost::mutex::scoped_lock lock(mutex_);
     // If there is more than one callback, force copy the messages
     bool nonconst_force_copy = callbacks_.size() > 1;
     for (auto& helper : callbacks_)
     {
       helper->call(nonconst_force_copy, events...);
     }
   }
 
 private:
   boost::mutex mutex_;
   CallbackHelperVector callbacks_;
 };
 
 } // namespace message_filters
 
 #endif // MESSAGE_FILTERS_SIGNAL_H
 