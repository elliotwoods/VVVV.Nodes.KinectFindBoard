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
	[PluginInfo(Name = "FullHouse", Category = "Kinect", Version = "Microsoft, Standalone", Author = "elliotwoods")]
	public class FullHouseNode : IPluginEvaluate, IPluginDXTexture2
	{
		[Input("Skeleton Smoothing", IsSingle = true)]
		IDiffSpread<TransformSmoothParameters> FInSkeletonSmoothing;

		[Input("Enabled", IsSingle=true)]
		IDiffSpread<bool> FInEnabled;

		[Output("Sensor")]
		ISpread<KinectSensor> FOutSensor;

		[Output("Skeletons")]
		ISpread<Skeleton> FOutSkeleton; 

		[Output("Frame Number")]
		ISpread<int> FOutFrameNumber;

		[Output("Timestamp", DimensionNames=new string[]{"s"})]
		ISpread<double> FOutTimestamp;

		[Output("Status")]
		ISpread<string> FOutStatus;

		private IDXTextureOut FOutColor;
		Dictionary<Device, Texture> FColorTexture = new Dictionary<Device, Texture>();
		private IDXTextureOut FOutDepth;
		Dictionary<Device, Texture> FDepthTexture = new Dictionary<Device, Texture>();
		private IDXTextureOut FOutWorld;
		Dictionary<Device, Texture> FWorldTexture = new Dictionary<Device, Texture>();


		private KinectSensor FSensor;
		
		private short[] FDepthData;
		private byte[] FColorData;
		private Skeleton[] FSkeletonData;

		private DepthImageFormat FDepthFormat;

		private float[] FWorldData1, FWorldData2;
		private bool FWorldDataPointer = false; // false = write2,read1

		private long FTimestamp;
		private int FFrameNumber;

		private bool FTimestampInvalidate = false;
		private bool FColorInvalidate = false;
		private bool FDepthInvalidate = false;
		private bool FSkeletonInvalidate = false;
		private bool FWorldInvalidate = false;

		private Object FColorLock = new Object();
		private Object FDepthLock = new Object();

		private ReaderWriterLock FWorldLock1 = new ReaderWriterLock();
		private ReaderWriterLock FWorldLock2 = new ReaderWriterLock();

		[ImportingConstructor()]
		public FullHouseNode(IPluginHost host)
		{
			host.CreateTextureOutput("Color", TSliceMode.Single, TPinVisibility.True, out this.FOutColor);
			host.CreateTextureOutput("Depth", TSliceMode.Single, TPinVisibility.True, out this.FOutDepth);
			host.CreateTextureOutput("World", TSliceMode.Single, TPinVisibility.True, out this.FOutWorld);
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

				FOutSensor[0] = FSensor; 
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

		void FSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
		{
			//color
			{
				var color = e.OpenColorImageFrame();
				if (color != null)
				{
					lock (FColorLock)
					{
						if (FColorData == null || FColorData.Length != color.PixelDataLength)
							FColorData = new byte[color.PixelDataLength];
						color.CopyPixelDataTo(FColorData);
					}

					FTimestamp = color.Timestamp;
					FFrameNumber = color.FrameNumber;
					FTimestampInvalidate = true;

					color.Dispose();
					FColorInvalidate = true;
				}
			}

			//depth
			{
				var depth = e.OpenDepthImageFrame();
				if (depth != null)
				{
					lock (FDepthLock)
					{
						if (FDepthData == null || FDepthData.Length != depth.PixelDataLength)
							FDepthData = new short[depth.PixelDataLength];
						depth.CopyPixelDataTo(FDepthData);
						FDepthFormat = depth.Format;
					}
					depth.Dispose();
					FDepthInvalidate = true;
				}
			}

			//skeletons
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
		}

		private unsafe void MakeWorld()
		{
			var writeBuffer = FWorldDataPointer ? FWorldData1 : FWorldData2;
			var writeLock = FWorldDataPointer ? FWorldLock1 : FWorldLock2;
			var width = FSensor.DepthStream.FrameWidth;
			var height = FSensor.DepthStream.FrameHeight;

			//writeLock.AcquireWriterLock(10);

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

				fixed (short* depthFixed = FDepthData)
				{
					fixed(float* worldFixed = writeBuffer)
					{
						short* depth = depthFixed;
						float* world = worldFixed;

						for(int y = 0; y<height; y++)
						{
							/*
							for (int x = 0; x < width; x++)
							{
								 var worldPoint = FSensor.MapDepthToSkeletonPoint(FDepthFormat, x, y, *depth++);
								*world++ = 1.0f;
								*world++ = worldPoint.X;
								*world++ = worldPoint.Y;
								*world++ = worldPoint.Z;
							}
							*/
							
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

				FWorldInvalidate = true;
				FWorldDataPointer = !FWorldDataPointer;
			}
			catch
			{

			}
			finally
			{
				//writeLock.ReleaseWriterLock();
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

						lock (FColorLock)
						{
							rectangle.Data.WriteRange(FColorData);
						}

						surface.UnlockRectangle();

						FColorInvalidate = false;
					}
				}

				//depth
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

						lock (FDepthLock)
						{
							rectangle.Data.WriteRange(FDepthData);
						}

						surface.UnlockRectangle();

						FDepthInvalidate = false;
						MakeWorld();
					}
				}

				//world
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
					if (FWorldInvalidate)
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

						FWorldInvalidate = false;
					}
				}

			}
		}
#endregion
	}
}
