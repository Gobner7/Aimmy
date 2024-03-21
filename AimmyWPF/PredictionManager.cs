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

        // Directional Change Prediction Method
        public Detection DirectionalChangePrediction(List<Detection> pastPositions) {
            if (pastPositions.Count < 3) return pastPositions.Last();

            var lastPosition = pastPositions[^1];
            var secondLastPosition = pastPositions[^2];
            var thirdLastPosition = pastPositions[^3];

            var direction1 = new { X = secondLastPosition.X - thirdLastPosition.X, Y = secondLastPosition.Y - thirdLastPosition.Y };
            var direction2 = new { X = lastPosition.X - secondLastPosition.X, Y = lastPosition.Y - secondLastPosition.Y };

            var nextStep = new {
                X = lastPosition.X + (direction2.X - direction1.X),
                Y = lastPosition.Y + (direction2.Y - direction1.Y)
            };

            return new Detection { X = nextStep.X, Y = nextStep.Y };
        }

        // Acceleration-Based Prediction Method
        public Detection AccelerationBasedPrediction(List<Detection> pastPositions) {
            if (pastPositions.Count < 3) return pastPositions.Last();

            var lastPosition = pastPositions[^1];
            var secondLastPosition = pastPositions[^2];
            var thirdLastPosition = pastPositions[^3];

            var velocity1 = new { X = secondLastPosition.X - thirdLastPosition.X, Y = secondLastPosition.Y - thirdLastPosition.Y };
            var velocity2 = new { X = lastPosition.X - secondLastPosition.X, Y = lastPosition.Y - secondLastPosition.Y };

            var acceleration = new {
                X = velocity2.X - velocity1.X,
                Y = velocity2.Y - velocity1.Y
            };

            var nextPosition = new {
                X = lastPosition.X + velocity2.X + acceleration.X,
                Y = lastPosition.Y + velocity2.Y + acceleration.Y
            };

            return new Detection { X = nextPosition.X, Y = nextPosition.Y };
        }

        // Adaptive Response Prediction Method
        private List<Detection> recentPredictions = new List<Detection>();

        public Detection AdaptiveResponsePrediction(List<Detection> pastPositions) {
            if (pastPositions.Count < 2 || recentPredictions.Count < 2) return pastPositions.Last();

            var lastActual = pastPositions.Last();
            var lastPrediction = recentPredictions.Last();

            var error = new {
                X = lastActual.X - lastPrediction.X,
                Y = lastActual.Y - lastPrediction.Y
            };

            var nextPrediction = new Detection {
                X = lastActual.X + error.X,
                Y = lastActual.Y + error.Y
            };

            recentPredictions.Add(nextPrediction);
            if (recentPredictions.Count > 10) // Keep the list size manageable
                recentPredictions.RemoveAt(0);

            return nextPrediction;
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

