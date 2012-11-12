using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using VVVV.MSKinect.Lib;
using VVVV.Utils.VMath;
using Microsoft.Kinect;

namespace VVVV.MSKinect.Nodes
{
	[PluginInfo(Name = "Kinect", Category = "Devices", Version = "Microsoft", Author = "elliotwoods", Credits="vux", Tags="High Res", AutoEvaluate=true)]
	public class HighResKinectRuntimeNode : IPluginEvaluate, IDisposable
	{
		[Input("Motor Angle", IsSingle = true, DefaultValue = 0.5)]
		IDiffSpread<double> FInAngle;

		[Input("Index", IsSingle = true)]
		IDiffSpread<int> FInIndex;
		
		[Input("Depth Range", IsSingle = true)]
		IDiffSpread<DepthRange> FInDepthRange;

		[Input("Enable Skeleton", IsSingle = true)]
		IDiffSpread<bool> FInEnableSkeleton;

		[Input("Enable Skeleton Smoothing", IsSingle = true, DefaultValue = 1)]
		IDiffSpread<bool> FInEnableSmooth;

		[Input("Smooth Parameters", IsSingle = true)]
		Pin<TransformSmoothParameters> FSmoothParams;

		[Input("Enabled", IsSingle = true)]
		IDiffSpread<bool> FInEnabled;

		[Input("Reset", IsBang = true)]
		ISpread<bool> FInReset;

		[Output("Kinect Runtime", IsSingle = true)]
		ISpread<KinectRuntime> FOutRuntime;

		[Output("Kinect Count", IsSingle = true)]
		ISpread<int> FOutKCnt;

		[Output("Kinect Status", IsSingle = true)]
		ISpread<KinectStatus> FOutStatus;

		[Output("Is Started", IsSingle = true)]
		ISpread<bool> FOutStarted;

		[Output("Color FOV")]
		ISpread<Vector2D> FOutColorFOV;

		[Output("Depth FOV")]
		ISpread<Vector2D> FOutDepthFOV;

		private KinectRuntime runtime = new KinectRuntime();

		private bool haskinect = false;

		public void Evaluate(int SpreadMax)
		{

			bool reset = false;

			if (this.FInIndex.IsChanged || this.FInReset[0] || this.runtime.Runtime == null)
			{
				this.haskinect = this.runtime.Assign(this.FInIndex[0]);
				reset = true;
			}

			if (this.haskinect)
			{

				if (this.FInEnabled.IsChanged || reset)
				{
					if (this.FInEnabled[0])
					{
						this.runtime.Start(true, this.FInEnableSkeleton[0], true);
						this.runtime.SetColor(true);
						this.runtime.SetDepthMode(true);
						this.runtime.SetSkeletonMode(SkeletonTrackingMode.Default);
						this.runtime.Runtime.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
						this.runtime.Runtime.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
					}
					else
					{
						this.runtime.Stop();
					}

					reset = true;
				}
				
				if (this.FInDepthRange.IsChanged || reset)
				{
					try
					{
						this.runtime.SetDepthRange(this.FInDepthRange[0]);
					}
					catch { }
				}


				if (this.FInEnableSkeleton.IsChanged || this.FInEnableSmooth.IsChanged || this.FSmoothParams.IsChanged || reset)
				{
					TransformSmoothParameters sp;
					if (this.FSmoothParams.PluginIO.IsConnected)
					{
						sp = this.FSmoothParams[0];
					}
					else
					{
						sp = this.runtime.DefaultSmooth();
					}

					this.runtime.EnableSkeleton(this.FInEnableSkeleton[0], this.FInEnableSmooth[0], sp);
				}

				if (this.FInAngle.IsChanged || reset)
				{
					if (this.runtime.IsStarted)
					{
						try { this.runtime.Runtime.ElevationAngle = (int)VMath.Map(this.FInAngle[0], 0, 1, this.runtime.Runtime.MinElevationAngle, this.runtime.Runtime.MaxElevationAngle, TMapMode.Clamp); }
						catch { }
					}
				}


				this.FOutStatus[0] = runtime.Runtime.Status;
				this.FOutRuntime[0] = runtime;
				this.FOutStarted[0] = runtime.IsStarted;

				this.FOutColorFOV.SliceCount = 1;
				this.FOutDepthFOV.SliceCount = 1;

				this.FOutColorFOV[0] = new Vector2D(this.runtime.Runtime.ColorStream.NominalHorizontalFieldOfView,
					this.runtime.Runtime.ColorStream.NominalVerticalFieldOfView);

				this.FOutDepthFOV[0] = new Vector2D(this.runtime.Runtime.DepthStream.NominalHorizontalFieldOfView,
					this.runtime.Runtime.DepthStream.NominalVerticalFieldOfView);
			}

			this.FOutKCnt[0] = KinectSensor.KinectSensors.Count;
		}

		public void Dispose()
		{
			if (this.runtime != null)
			{
				this.runtime.Stop();
				this.runtime.Runtime.Dispose();
			}
		}
	}
}
