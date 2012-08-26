using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using VVVV.PluginInterfaces.V1;
using VVVV.Utils.VMath;
using System.Runtime.InteropServices;
using VVVV.MSKinect.Lib;
using System.ComponentModel.Composition;
using SlimDX.Direct3D9;
using SlimDX;
using Microsoft.Kinect;

using VVVV.Nodes.OpenCV;

using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.CvEnum;
using System.Drawing;

namespace VVVV.MSKinect.Nodes
{
	[PluginInfo(Name = "FindBoard", Category = "Kinect", Version = "Microsoft", Author = "vux", Tags = "directx,texture")]
	public class FindBoardNode : IPluginEvaluate, IPluginConnections
	{
		[Input("Kinect Runtime")]
		private Pin<KinectRuntime> FInRuntime;

		[Input("Board Size X")]
		private IDiffSpread<int> FInBoardX;

		[Input("Board Size X")]
		private IDiffSpread<int> FInBoardY;

		[Output("Corners RGB Map")]
		private ISpread<Vector2D> FOutCornersRGB;

		[Output("Corners Depth Map")]
		private ISpread<Vector2D> FOutCornersDepth;

		[Output("Corners World")]
		private ISpread<Vector3D> FOutCornersXYZ;

		[Output("Frame Index", IsSingle = true, Order = 10)]
		private ISpread<int> FOutFrameIndex;

		private int frameindex = -1;

		private bool FInvalidateConnect = false;
		private bool FInvalidate = true;

		private KinectRuntime runtime;

		private byte[] FDataRGB;
		private CVImage FImageRGB = new CVImage();
		private CVImage FImageLuminance = new CVImage();
		private short[] FDataDepth;
		private ColorImagePoint[] FMappingDepthToColor = null;
		private int[] FMappingColorToDepth = null;

		private object FLockDepth = new object();
		private object FLockRGB = new object();

		private System.Drawing.Size FBoardSize = new System.Drawing.Size();
		private object FLockBoard = new object();

		[ImportingConstructor()]
		public FindBoardNode(IPluginHost host)
		{
		}

		public void Evaluate(int SpreadMax)
		{
			if (FInBoardX.IsChanged || FInBoardY.IsChanged)
				MakeBoard();

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

		private void MakeBoard()
		{
			lock (this.FLockBoard)
			{
				this.FBoardSize.Width = FInBoardX[0];
				this.FBoardSize.Height = FInBoardY[0];
			}
		}

		private int depthFrameAvailable = -1;

		private void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
		{
			DepthImageFrame frame = e.OpenDepthImageFrame();

			if (frame != null)
			{
				lock (FLockDepth)
				{
					this.depthFrameAvailable = frame.FrameNumber;
					if (FDataDepth == null || FDataDepth.Length != frame.PixelDataLength)
						FDataDepth = new short[frame.PixelDataLength];
					frame.CopyPixelDataTo(this.FDataDepth);

					if (FMappingDepthToColor == null || FMappingDepthToColor.Length != frame.PixelDataLength)
						FMappingDepthToColor = new ColorImagePoint[frame.PixelDataLength];


					DepthImageFormat depthFormat = runtime.Runtime.DepthStream.Format;
					ColorImageFormat colorFormat = runtime.Runtime.ColorStream.Format;

					this.runtime.Runtime.MapDepthFrameToColorFrame(depthFormat, FDataDepth, colorFormat, FMappingDepthToColor);
				}
				this.FInvalidate = true;
				checkFindBoards();
			}
		}

		private int rgbFrameAvailable = -1;

		private void RGBFrameReady(object sender, ColorImageFrameReadyEventArgs e)
		{
			ColorImageFrame frame = e.OpenColorImageFrame();

			if (frame != null)
			{
				this.rgbFrameAvailable = frame.FrameNumber;

				//allocate buffer
				if (FDataRGB == null || FDataRGB.Length != frame.Width * frame.Height)
					FDataRGB = new byte[frame.Width * frame.Height];

				lock (FLockRGB)
				{
					//allocate image
					if (FImageRGB.Width != frame.Width || FImageRGB.Height != frame.Height)
						FImageRGB.Initialise(new System.Drawing.Size(frame.Width, frame.Height), TColorFormat.RGB8);
					if (FImageLuminance == null || FImageLuminance.Size.Width != frame.Width || FImageLuminance.Size.Height != frame.Height)
						FImageLuminance.Initialise(new System.Drawing.Size(frame.Width, frame.Height), TColorFormat.L8);

					frame.CopyPixelDataTo(FDataRGB);
					FImageRGB.SetPixels(FDataRGB);
					FImageRGB.GetImage(FImageLuminance);
				}
				this.FInvalidate = true;
				checkFindBoards();
			}
		}

		object FLockPoints = new object();
		Spread<Vector2D> FPointsRGBMap = new Spread<Vector2D>();
		Spread<Vector2D> FPointsDepthMap = new Spread<Vector2D>();
		Spread<Vector3D> FPointsWorld = new Spread<Vector3D>();

		private void checkFindBoards()
		{
			//if we've got matching depth and rgb frames then let's find some chessboards!
			if (this.rgbFrameAvailable == this.depthFrameAvailable)
			{
				PointF[] pointsRGB;
				lock (FLockRGB)
				{
					var image = FImageLuminance.GetImage() as Image<Gray, byte>;
					pointsRGB = CameraCalibration.FindChessboardCorners(image, FBoardSize, CALIB_CB_TYPE.ADAPTIVE_THRESH);
				}

				PointF[] pointsDepthMap = new PointF[pointsRGB.Length];
				Vector3D[] pointsWorld = new Vector3D[pointsRGB.Length];
				ushort[] pointsDepthValue = new ushort[pointsRGB.Length];
				lock (FLockDepth)
				{
					invertLookup();
					int idxRGB, idxDepth;
					int depthWidth = this.runtime.Runtime.DepthStream.FrameWidth;
					ushort depth;
					for (int i = 0; i < pointsRGB.Length; i++)
					{
						idxRGB = (int) pointsRGB[i].X + (int)(pointsRGB[i].Y * (float) this.runtime.Runtime.ColorStream.FrameWidth);
						if (idxRGB > this.runtime.Runtime.ColorStream.FramePixelDataLength)
							continue;
						idxDepth = FMappingColorToDepth[idxRGB];
						pointsDepthMap[i] = new PointF(idxDepth % depthWidth, idxDepth / depthWidth);
						depth = 
					}
				}

				lock (FLockPoints)
				{
					FPointsRGBMap.SliceCount = pointsRGB.Length;
					FPointsDepthMap.SliceCount = pointsRGB.Length;
					FPointsWorld.SliceCount = pointsRGB.Length;

				}

			}
		}

		private void invertLookup()
		{
			if (FMappingColorToDepth == null || FMappingColorToDepth.Length != runtime.Runtime.ColorStream.FramePixelDataLength)
				FMappingColorToDepth = new int[runtime.Runtime.ColorStream.FramePixelDataLength];

			int stride = runtime.Runtime.ColorStream.FrameWidth;

			int idx = 0;
			foreach (var depthToColor in FMappingDepthToColor)
			{
				FMappingColorToDepth[depthToColor.X + stride * depthToColor.Y] = idx++;
			}
		}
	}
}
