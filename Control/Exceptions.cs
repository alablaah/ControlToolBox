using System;

namespace ControlToolBox
{
    public class InvalidDimensionsException : Exception
    {
        public InvalidDimensionsException() : base() { }
        public InvalidDimensionsException(string message) : base(message) { }
        public InvalidDimensionsException(string message, Exception inner) : base(message, inner) { }

        // A constructor is needed for serialization when an
        // exception propagates from a remoting server to the client.
        protected InvalidDimensionsException(System.Runtime.Serialization.SerializationInfo info,
            System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }

    public class AliasingException : Exception
    {
        public AliasingException() : base() { }
        public AliasingException(string message) : base(message) { }
        public AliasingException(string message, Exception inner) : base(message, inner) { }

        // A constructor is needed for serialization when an
        // exception propagates from a remoting server to the client.
        protected AliasingException(System.Runtime.Serialization.SerializationInfo info,
            System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }

    public class SamplingRateRequiredException : Exception
    {
        public SamplingRateRequiredException() : base() { }
        public SamplingRateRequiredException(string message) : base(message) { }
        public SamplingRateRequiredException(string message, Exception inner) : base(message, inner) { }

        // A constructor is needed for serialization when an
        // exception propagates from a remoting server to the client.
        protected SamplingRateRequiredException(System.Runtime.Serialization.SerializationInfo info,
            System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }

}
