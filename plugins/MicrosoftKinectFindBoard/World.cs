using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using VVVV.PluginInterfaces.V1;
using System.Runtime.InteropServices;
using VVVV.MSKinect.Lib;
using System.ComponentModel.Composition;
using SlimDX.Direct3D9;
using SlimDX;
using Microsoft.Kinect;

namespace VVVV.MSKinect.Nodes
{
	[PluginInfo(Name = "World", Category = "Kinect", Version = "Microsoft", Author = "elliotwoods", Credits = "vux", Tags = "directx,texture")]
	public class KinectWorldTextureNode : IPluginEvaluate, IPluginConnections, IPluginDXTexture2
	{
		[Input("Kinect Runtime")]
		private Pin<KinectRuntime> FInRuntime;

		private IDXTextureOut FOutTexture;

		[Output("Frame Index", IsSingle = true, Order = 10)]
		private ISpread<int> FOutFrameIndex;

		private int frameindex = -1;

		private bool FInvalidateConnect = false;
		private bool FInvalidate = true;

		private KinectRuntime runtime;

		private short[] rawdepth;
		private float[] worldImage;

		private object m_lock = new object();

		private Dictionary<Device, Texture> FDepthTex = new Dictionary<Device, Texture>();

		[ImportingConstructor()]
		public KinectWorldTextureNode(IPluginHost host)
		{
			host.CreateTextureOutput("Texture Out", TSliceMode.Single, TPinVisibility.True, out this.FOutTexture);
		}

		public void Evaluate(int SpreadMax)
		{
			if (this.FInvalidateConnect)
			{
				if (runtime != null)
				{
					this.runtime.DepthFrameReady -= DepthFrameReady;
				}

				if (this.FInRuntime.PluginIO.IsConnected)
				{
					//Cache runtime node
					this.runtime = this.FInRuntime[0];

					if (runtime != null)
					{
						this.FInRuntime[0].DepthFrameReady += DepthFrameReady;
					}

				}

				this.FInvalidateConnect = false;
			}

			this.FOutFrameIndex[0] = this.frameindex;
		}

		public void ConnectPin(IPluginIO pin)
		{
			if (pin == this.FInRuntime.PluginIO)
			{
				this.FInvalidateConnect = true;
			}
		}

		public void DisconnectPin(IPluginIO pin)
		{
			if (pin == this.FInRuntime.PluginIO)
			{
				this.FInvalidateConnect = true;
			}
		}

		public Texture GetTexture(IDXTextureOut ForPin, Device OnDevice, int Slice)
		{
			if (this.FDepthTex.ContainsKey(OnDevice))
			{
				return this.FDepthTex[OnDevice];
			}
			else
				return null;
		}

		public void UpdateResource(IPluginOut ForPin, Device OnDevice)
		{
			if (this.runtime != null && this.worldImage != null)
			{
				var width = runtime.Runtime.DepthStream.FrameWidth;
				var height = runtime.Runtime.DepthStream.FrameHeight;

				if (!this.FDepthTex.ContainsKey(OnDevice))
				{
					Texture t = null;
					if (OnDevice is DeviceEx)
					{
						t = new Texture(OnDevice, width, height, 1, Usage.None, Format.A32B32G32R32F, Pool.Default);
					}
					else
					{
						t = new Texture(OnDevice, width, height, 1, Usage.None, Format.A32B32G32R32F, Pool.Managed);
					}
					this.FDepthTex.Add(OnDevice, t);
				}

				if (this.FInvalidate)
				{
					Texture tx = this.FDepthTex[OnDevice];
					Surface srf = tx.GetSurfaceLevel(0);
					DataRectangle rect = srf.LockRectangle(LockFlags.Discard);

					lock (this.m_lock)
					{
						rect.Data.WriteRange(this.worldImage);
					}
					srf.UnlockRectangle();


					this.FInvalidate = false;
				}
			}
		}

		public void DestroyResource(IPluginOut ForPin, Device OnDevice, bool OnlyUnManaged)
		{
			if (this.FDepthTex.ContainsKey(OnDevice))
			{
				this.FDepthTex[OnDevice].Dispose();
				this.FDepthTex.Remove(OnDevice);
			}
		}

		private void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
		{
			DepthImageFrame frame = e.OpenDepthImageFrame();

			if (frame != null)
			{
				if (this.rawdepth == null || this.rawdepth.Length != frame.PixelDataLength)
				{
					this.rawdepth = new short[frame.PixelDataLength];
					this.worldImage = new float[frame.PixelDataLength * 4];
				}

				this.FInvalidate = true;
				this.frameindex = frame.FrameNumber;

				frame.CopyPixelDataTo(this.rawdepth);
				makeWorld();
			}
		}

		private unsafe void makeWorld()
		{
			var depthStream = this.runtime.Runtime.DepthStream;

			fixed (float* fixedWorld = worldImage)
			{
				fixed (short* fixedDepth = rawdepth)
				{
					short* depth = fixedDepth;
					float* world = fixedWorld;

					for (int j = 0; j < depthStream.FrameHeight; j++)
					{
						for (int i = 0; i < depthStream.FrameWidth; i++)
						{
							var xyz = this.runtime.Runtime.MapDepthToSkeletonPoint(depthStream.Format, i, j, *depth++);

							*world++ = xyz.X;
							*world++ = xyz.Y;
							*world++ = xyz.Z;
							*world++ = 1.0f;
						}
					}
				}
			}
		}
	}
}
