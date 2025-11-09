using System.Diagnostics; 
using System;

namespace AmbientOcclusion.Geometry.Scripts

{
    public static class PerformanceUtils
    {
 
        public static TimeSpan MeasureExecutionTime(Action actionToMeasure)
        {
            if (actionToMeasure == null)
            {
                throw new ArgumentNullException(nameof(actionToMeasure));
            }

            Stopwatch stopwatch = Stopwatch.StartNew(); 
            actionToMeasure();
            stopwatch.Stop();

            return stopwatch.Elapsed;
        }
        
        public static void MeasureAndLogMs(string description, Action actionToMeasure)
        {
            TimeSpan elapsed = MeasureExecutionTime(actionToMeasure);
            UnityEngine.Debug.Log($"{description} took: {elapsed.TotalMilliseconds:F4} ms");
        }
        
        public static void MeasureAndLogSec(string description, Action actionToMeasure)
        {
            TimeSpan elapsed = MeasureExecutionTime(actionToMeasure);
            UnityEngine.Debug.Log($"{description} took: {elapsed.TotalSeconds:F6} s");
        }
    }
}