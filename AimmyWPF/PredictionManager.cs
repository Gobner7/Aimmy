using Accord.Statistics.Running;
using System;

namespace AimmyWPF
{
    internal class PredictionManager
    {
        public struct Detection
        {
            public int X;
            public int Y;
            public DateTime Timestamp;
        }

        private KalmanFilter2D kalmanFilter;
        private DateTime lastUpdateTime;

        public PredictionManager()
        {
            kalmanFilter = new KalmanFilter2D();
            lastUpdateTime = DateTime.UtcNow;
        }

        public void UpdateKalmanFilter(Detection detection)
        {
            var currentTime = DateTime.UtcNow;

            kalmanFilter.Push(detection.X, detection.Y);
            lastUpdateTime = currentTime;
        }

        public Detection GetEstimatedPosition()
        {
            // Current estimated position
            double currentX = kalmanFilter.X;
            double currentY = kalmanFilter.Y;

            // Current velocity
            double velocityX = kalmanFilter.XAxisVelocity;
            double velocityY = kalmanFilter.YAxisVelocity;

            // Calculate time since last update
            double timeStep = (DateTime.UtcNow - lastUpdateTime).TotalSeconds;

            // Predict next position based on current position and velocity
            double predictedX = currentX + velocityX * timeStep;
            double predictedY = currentY + velocityY * timeStep;

            return new Detection { X = (int)predictedX, Y = (int)predictedY };
        }
    }

        // Simple Moving Average Prediction Method
        public Detection SimpleMovingAveragePrediction(List<Detection> pastPositions, int windowSize) {
            if (pastPositions.Count < windowSize) return pastPositions.Last();

            int sumX = 0, sumY = 0;
            for (int i = pastPositions.Count - windowSize; i < pastPositions.Count; i++) {
                sumX += pastPositions[i].X;
                sumY += pastPositions[i].Y;
            }

            return new Detection { X = sumX / windowSize, Y = sumY / windowSize };
        }

        // Median Filter Prediction Method
        public Detection MedianFilterPrediction(List<Detection> pastPositions) {
            if (pastPositions.Count < 3) return pastPositions.Last();

            List<int> xValues = pastPositions.Select(p => p.X).ToList();
            List<int> yValues = pastPositions.Select(p => p.Y).ToList();
            xValues.Sort();
            yValues.Sort();

            int medianX = xValues[xValues.Count / 2];
            int medianY = yValues[yValues.Count / 2];

            return new Detection { X = medianX, Y = medianY };
        }

        // Mode Filter Prediction Method (Most frequent value)
        public Detection ModeFilterPrediction(List<Detection> pastPositions) {
            if (!pastPositions.Any()) return new Detection { X = 0, Y = 0 };

            var xMode = pastPositions.GroupBy(p => p.X).OrderByDescending(g => g.Count()).First().Key;
            var yMode = pastPositions.GroupBy(p => p.Y).OrderByDescending(g => g.Count()).First().Key;

            return new Detection { X = xMode, Y = yMode };
        }

}

        // Simple Moving Average Prediction Method
        public Detection SimpleMovingAveragePrediction(List<Detection> pastPositions, int windowSize) {
            if (pastPositions.Count < windowSize) return pastPositions.Last();

            int sumX = 0, sumY = 0;
            for (int i = pastPositions.Count - windowSize; i < pastPositions.Count; i++) {
                sumX += pastPositions[i].X;
                sumY += pastPositions[i].Y;
            }

            return new Detection { X = sumX / windowSize, Y = sumY / windowSize };
        }

        // Median Filter Prediction Method
        public Detection MedianFilterPrediction(List<Detection> pastPositions) {
            if (pastPositions.Count < 3) return pastPositions.Last();

            List<int> xValues = pastPositions.Select(p => p.X).ToList();
            List<int> yValues = pastPositions.Select(p => p.Y).ToList();
            xValues.Sort();
            yValues.Sort();

            int medianX = xValues[xValues.Count / 2];
            int medianY = yValues[yValues.Count / 2];

            return new Detection { X = medianX, Y = medianY };
        }

        // Mode Filter Prediction Method (Most frequent value)
        public Detection ModeFilterPrediction(List<Detection> pastPositions) {
            if (!pastPositions.Any()) return new Detection { X = 0, Y = 0 };

            var xMode = pastPositions.GroupBy(p => p.X).OrderByDescending(g => g.Count()).First().Key;
            var yMode = pastPositions.GroupBy(p => p.Y).OrderByDescending(g => g.Count()).First().Key;

            return new Detection { X = xMode, Y = yMode };
        }

