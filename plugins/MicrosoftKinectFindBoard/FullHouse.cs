using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using VVVV.PluginInterfaces.V1;
using Microsoft.Kinect;
using System.ComponentModel.Composition;
using SlimDX.Direct3D9;
using System.Threading;
using System.Threading.Tasks;

namespace VVVV.Nodes.MSKinect
{
	class FullHouseContext
	{
		public FullHouseNode FullHouseNode;

		public FullHouseContext(FullHouseNode node)
		{
			this.FullHouseNode = node;
		}
	}

	[PluginInfo(Name = "FullHouse", Category = "Kinect", Version = "Microsoft, Standalone", Author = "elliotwoods")]
	public class FullHouseNode : IPluginEvaluate, IPluginDXTexture2, IDisposable
	{
		[Input("Skeleton Smoothing", IsSingle = true)]
		IDiffSpread<TransformSmoothParameters> FInSkeletonSmoothing;

		[Input("Image Registration", IsSingle = true)]
		IDiffSpread<bool> FInRegistration;

		[Input("Enabled", IsSingle=true)]
		IDiffSpread<bool> FInEnabled;

		[Output("Context")]
		ISpread<FullHouseContext> FOutContext;

		[Output("Frame Number")]
		ISpread<int> FOutFrameNumber;

		[Output("Timestamp", DimensionNames=new string[]{"s"})]
		ISpread<double> FOutTimestamp;

		[Output("Skeletons")]
		Pin<Skeleton> FOutSkeleton; 

		[Output("Status", Order=1000)]
		ISpread<string> FOutStatus;

		private IDXTextureOut FOutColor;
		Dictionary<Device, Texture> FColorTexture = new Dictionary<Device, Texture>();
		private IDXTextureOut FOutDepth;
		Dictionary<Device, Texture> FDepthTexture = new Dictionary<Device, Texture>();
		private IDXTextureOut FOutWorld;
		Dictionary<Device, Texture> FWorldTexture = new Dictionary<Device, Texture>();

		public KinectSensor FSensor;
		
		public short[] DepthData;
		private byte[] FUnregisteredColorData;
		public byte[] ColorData;
		private Skeleton[] FSkeletonData;
		private ColorImagePoint[] FRegistrationData;

		private DepthImageFormat FDepthFormat;

		private float[] FWorldData1, FWorldData2;
		private bool FWorldDataPointer = false; // false = write2,read1
		public float[] WorldData
		{
			get
			{
				if (FWorldDataPointer)
					return FWorldData2;
				else
					return FWorldData1;
			}
		}

		private long FTimestamp;
		private int FFrameNumber;

		private bool FTimestampInvalidate = false;
		private bool FColorInvalidate = false;
		private bool FDepthInvalidate = false;
		private bool FSkeletonInvalidate = false;
		private bool FWorldInInvalidate = false;
		private bool FWorldOutInvalidate = false;

		public Object ColorLock = new Object();
		public Object DepthLock = new Object();

		private ReaderWriterLock FWorldLock1 = new ReaderWriterLock();
		private ReaderWriterLock FWorldLock2 = new ReaderWriterLock();
		public ReaderWriterLock FWorldLock
		{
			get
			{
				if (FWorldDataPointer)
					return FWorldLock1;
				else
					return FWorldLock2;
			}
		}
		Thread FWorldThread;
		bool FWorldThreadRunning = true;

		public event EventHandler Update;

		public void OnUpdate()
		{
			if (Update == null)
				return;
			Update(this, EventArgs.Empty);
		}

		[ImportingConstructor()]
		public FullHouseNode(IPluginHost host)
		{
			host.CreateTextureOutput("Color", TSliceMode.Single, TPinVisibility.True, out this.FOutColor);
			host.CreateTextureOutput("Depth", TSliceMode.Single, TPinVisibility.True, out this.FOutDepth);
			host.CreateTextureOutput("World", TSliceMode.Single, TPinVisibility.True, out this.FOutWorld);
			FWorldThread = new Thread(WorldThread);
			FWorldThread.Start();
		}

		public void Evaluate(int SpreadMax)
		{
			if (FInEnabled.IsChanged)
			{
				if (FInEnabled[0])
				{
					try
					{
						FSensor = FindSensor();
						FSensor.Start();
						FSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
						FSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
						EnableSkeletonStream();							
						FSensor.AllFramesReady += FSensor_AllFramesReady;
					}
					catch (Exception e)
					{
						if (FSensor != null)
						{
							FSensor.Dispose();
							FSensor = null;
						}
						FOutStatus[0] = e.Message;
					}
				}
				else
				{
					if (FSensor != null)
					{
						FSensor.AllFramesReady -= FSensor_AllFramesReady;
						FSensor.Stop();
						FSensor.Dispose();
						FSensor = null;
					}
				}

				FOutContext[0] = new FullHouseContext(this);
			}

			if (FInSkeletonSmoothing.IsChanged)
			{
				if (FSensor != null)
				{
					EnableSkeletonStream();
				}
			}

			if (FTimestampInvalidate)
			{
				FOutFrameNumber[0] = FFrameNumber;
				FOutTimestamp[0] = ( (double) FTimestamp) / 1000.0;
				FTimestampInvalidate = false;
			}

			if (FSkeletonInvalidate)
			{
				FOutSkeleton.SliceCount = 0;
				foreach (var skeleton in FSkeletonData)
				{
					if (skeleton.TrackingId > 0)
						FOutSkeleton.Add(skeleton);
				}
			}
		}

		private void EnableSkeletonStream()
		{
			if (FInSkeletonSmoothing[0] != null)
				FSensor.SkeletonStream.Enable(FInSkeletonSmoothing[0]);
			else
				FSensor.SkeletonStream.Enable();
		}

		private KinectSensor FindSensor()
		{
			foreach (var sensor in KinectSensor.KinectSensors)
			{
				return sensor;
			}
			throw (new Exception("No Kinect sensor found."));
		}

		void ClearColorData()
		{
			//seperate function here to wrap lambda function, so can debug larger function easier
			Parallel.For(0, ColorData.Length, i => ColorData[i] = 0);
		}

		void RemapColors()
		{
			var width = FSensor.ColorStream.FrameWidth;
			var height = FSensor.ColorStream.FrameHeight;

			//seperate function here to wrap lambda function, so can debug larger function easier
			Parallel.For(0, width * height, i =>
			{
				var index = FRegistrationData[i].X + FRegistrationData[i].Y * width;

				if (index < width * height)
				{
					ColorData[i * 4] = FUnregisteredColorData[index * 4];
					ColorData[i * 4 + 1] = FUnregisteredColorData[index * 4 + 1];
					ColorData[i * 4 + 2] = FUnregisteredColorData[index * 4 + 2];
					ColorData[i * 4 + 3] = FUnregisteredColorData[index * 4 + 3];
				}
			});
		}

		void FSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
		{
			//if we have listeners, then make all data anyway
			bool DoAnyway = Update != null;

			//cache a value across whole function
			var UseRegistration = FInRegistration[0] || DoAnyway;

			//depth
			if (FOutDepth.IsConnected || FOutWorld.IsConnected || DoAnyway)
			{
				var depth = e.OpenDepthImageFrame();
				if (depth != null)
				{
					lock (DepthLock)
					{
						if (DepthData == null || DepthData.Length != depth.PixelDataLength)
							DepthData = new short[depth.PixelDataLength];

						depth.CopyPixelDataTo(DepthData);
						FDepthFormat = depth.Format;
					}
					depth.Dispose();
					FDepthInvalidate = true;

					if (FOutWorld.IsConnected)
						FWorldInInvalidate = true;

					if (UseRegistration)
					{
						if (FRegistrationData == null || DepthData.Length != FRegistrationData.Length)
							FRegistrationData = new ColorImagePoint[DepthData.Length];

						var depthFormat = FSensor.DepthStream.Format;
						var colorFormat = FSensor.ColorStream.Format;
						FSensor.MapDepthFrameToColorFrame(depthFormat, DepthData, colorFormat, FRegistrationData);
					}
				}
			}

			//color
			if (FOutColor.IsConnected || DoAnyway)
			{
				var color = e.OpenColorImageFrame();
				if (color != null)
				{
					if (UseRegistration)
					{
						if (FUnregisteredColorData == null || FUnregisteredColorData.Length != color.PixelDataLength)
							FUnregisteredColorData = new byte[color.PixelDataLength];
						color.CopyPixelDataTo(FUnregisteredColorData);

						lock (ColorLock)
						{
							if (ColorData == null || ColorData.Length != color.PixelDataLength)
								ColorData = new byte[color.PixelDataLength];

							//clear output
							ClearColorData();

							//remap colors
							RemapColors();
						}
					}
					else
					{
						lock (ColorLock)
						{
							if (ColorData == null || ColorData.Length != color.PixelDataLength)
								ColorData = new byte[color.PixelDataLength];
							color.CopyPixelDataTo(ColorData);
						}
					}

					FTimestamp = color.Timestamp;
					FFrameNumber = color.FrameNumber;
					FTimestampInvalidate = true;

					color.Dispose();
					FColorInvalidate = true;
				}
			}

			//skeletons
			if (FOutSkeleton.PluginIO.IsConnected || DoAnyway)
			{
				var skeletons = e.OpenSkeletonFrame();
				if (skeletons != null)
				{
					if (FSkeletonData == null || FSkeletonData.Length != skeletons.SkeletonArrayLength)
						FSkeletonData = new Skeleton[skeletons.SkeletonArrayLength];
					skeletons.CopySkeletonDataTo(FSkeletonData);
					skeletons.Dispose();
					FSkeletonInvalidate = true;
				}
			}

			OnUpdate();
		}

		private void WorldThread()
		{
			while (FWorldThreadRunning)
			{
				if (FWorldInInvalidate)
				{
					MakeWorld();
					FWorldInInvalidate = false;
				}
				else
				{
					Thread.Sleep(1);
				}
			}
		}

		short[] WorldDepthCopy;
		private unsafe void MakeWorld()
		{
			var writeBuffer = FWorldDataPointer ? FWorldData1 : FWorldData2;
			var writeLock = FWorldDataPointer ? FWorldLock1 : FWorldLock2;
			var width = FSensor.DepthStream.FrameWidth;
			var height = FSensor.DepthStream.FrameHeight;

			lock (DepthLock)
			{
				if (WorldDepthCopy == null || WorldDepthCopy.Length != DepthData.Length)
					WorldDepthCopy = new short[DepthData.Length];
				Array.Copy(DepthData, WorldDepthCopy, DepthData.Length);
			}

			writeLock.AcquireWriterLock(10);

			try
			{
				if (writeBuffer == null || writeBuffer.Length != width * height * 4)
				{
					if (FWorldDataPointer)
						FWorldData1 = new float[width * height * 4];
					else
						FWorldData2 = new float[width * height * 4];
					writeBuffer = FWorldDataPointer ? FWorldData1 : FWorldData2;
				}

				fixed (short* depthFixed = WorldDepthCopy)
				{
					fixed(float* worldFixed = writeBuffer)
					{
						short* depth = depthFixed;
						float* world = worldFixed;

						for(int y = 0; y<height; y++)
						{							
							Parallel.For(0, width, (x) =>
							{
								var worldPoint = FSensor.MapDepthToSkeletonPoint(FDepthFormat, x, y, depth[x]);
								world[x * 4 + 0] = worldPoint.X;
								world[x * 4 + 1] = worldPoint.Y;
								world[x * 4 + 2] = worldPoint.Z;
								world[x * 4 + 3] = 1.0f;
							});

							depth += width;
							world += width * 4;
						}

					}
				}

				FWorldDataPointer = !FWorldDataPointer;
				FWorldOutInvalidate = true;
			}
			catch
			{

			}
			finally
			{
				writeLock.ReleaseWriterLock();
			}
		}

#region Texture
		public Texture GetTexture(IDXTextureOut ForPin, Device OnDevice, int Slice)
		{
			if (ForPin == FOutColor)
				return FindTexture(FColorTexture, OnDevice);

			if (ForPin == FOutDepth)
				return FindTexture(FDepthTexture, OnDevice);

			if (ForPin == FOutWorld)
				return FindTexture(FWorldTexture, OnDevice);

			return null;
		}

		private Texture FindTexture(Dictionary<Device, Texture> TextureDictionary, Device Device)
		{
			if (TextureDictionary.ContainsKey(Device))
				return TextureDictionary[Device];
			else
				return null;
		}

		public void DestroyResource(IPluginOut ForPin, Device OnDevice, bool OnlyUnManaged)
		{
			if (ForPin == FOutColor)
				DestroyTexture(FColorTexture, OnDevice);

			if (ForPin == FOutDepth)
				DestroyTexture(FDepthTexture, OnDevice);

			if (ForPin == FOutWorld)
				DestroyTexture(FWorldTexture, OnDevice);
		}

		private void DestroyTexture(Dictionary<Device, Texture> TextureDictionary, Device Device)
		{
			if (TextureDictionary.ContainsKey(Device))
			{
				TextureDictionary[Device].Dispose();
				TextureDictionary.Remove(Device);
			}
		}

		public void UpdateResource(IPluginOut ForPin, Device OnDevice)
		{
			if (this.FSensor != null)
			{
				//color
				if (FOutColor.IsConnected)
				{
					var width = this.FSensor.ColorStream.FrameWidth;
					var height = this.FSensor.ColorStream.FrameHeight;

					//create if necessary
					if (!FColorTexture.ContainsKey(OnDevice))
					{
						var texture = new Texture(OnDevice, width, height, 1, Usage.None, Format.X8R8G8B8, (OnDevice is DeviceEx) ? Pool.Default : Pool.Managed);
						FColorTexture.Add(OnDevice, texture);
					}

					//fill
					if (FColorInvalidate)
					{
						var texture = FColorTexture[OnDevice];
						var surface = texture.GetSurfaceLevel(0);
						var rectangle = surface.LockRectangle(LockFlags.Discard);

						lock (ColorLock)
						{
							rectangle.Data.WriteRange(ColorData);
						}

						surface.UnlockRectangle();

						FColorInvalidate = false;
					}
				}

				//world
				if (FOutWorld.IsConnected)
				{
					var width = FSensor.DepthStream.FrameWidth;
					var height = FSensor.DepthStream.FrameHeight;

					//create if necessary
					if (!FWorldTexture.ContainsKey(OnDevice))
					{
						var texture = new Texture(OnDevice, width, height, 1, Usage.None, Format.A32B32G32R32F, (OnDevice is DeviceEx) ? Pool.Default : Pool.Managed);
						FWorldTexture.Add(OnDevice, texture);
					}

					//fill
					if (FWorldOutInvalidate)
					{
						var texture = FWorldTexture[OnDevice];
						var surface = texture.GetSurfaceLevel(0);
						var rectangle = surface.LockRectangle(LockFlags.Discard);

						var worldData = FWorldDataPointer ? FWorldData2 : FWorldData1;
						var worldLock = FWorldDataPointer ? FWorldLock2 : FWorldLock1;

						worldLock.AcquireReaderLock(100);
						try
						{
							rectangle.Data.WriteRange(worldData);
						}
						catch
						{

						}
						finally
						{
							worldLock.ReleaseReaderLock();
						}

						surface.UnlockRectangle();

						FWorldOutInvalidate = false;
					}
				}

				//depth
				if (FOutDepth.IsConnected)
				{
					var width = FSensor.DepthStream.FrameWidth;
					var height = FSensor.DepthStream.FrameHeight;

					//create if necessary
					if (!FDepthTexture.ContainsKey(OnDevice))
					{
						var texture = new Texture(OnDevice, width, height, 1, Usage.None, Format.L16, (OnDevice is DeviceEx) ? Pool.Default : Pool.Managed);
						FDepthTexture.Add(OnDevice, texture);
					}

					//fill
					if (FDepthInvalidate)
					{
						var texture = FDepthTexture[OnDevice];
						var surface = texture.GetSurfaceLevel(0);
						var rectangle = surface.LockRectangle(LockFlags.Discard);

						lock (DepthLock)
						{
							rectangle.Data.WriteRange(DepthData);
						}

						surface.UnlockRectangle();

						FDepthInvalidate = false;
					}
				}

			}
		}
#endregion

		public void Dispose()
		{
			if (FSensor != null)
			{
				if (FSensor.IsRunning)
					FSensor.Stop();
				FSensor.Dispose();
			}
			if (FWorldThread != null)
			{
				FWorldThreadRunning = false;
				FWorldThread.Join();
			}
		}
	}
}
