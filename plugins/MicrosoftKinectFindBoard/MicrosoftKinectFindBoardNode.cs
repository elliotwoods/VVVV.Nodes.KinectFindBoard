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
	[PluginInfo(Name = "FindBoard", Category = "Kinect", Version = "Microsoft", Author = "elliotwoods", Credits = "vux", Tags = "directx,texture")]
	public class FindBoardNode : IPluginEvaluate, IPluginConnections
	{
		#region pins and fields
		[Input("Kinect Runtime")]
		private Pin<KinectRuntime> FInRuntime;

		[Input("Board Size X", DefaultValue=10, MinValue=1)]
		private IDiffSpread<int> FInBoardX;

		[Input("Board Size Y", DefaultValue = 7, MinValue = 1)]
		private IDiffSpread<int> FInBoardY;

		[Input("Do", IsBang = true)]
		private ISpread<bool> FInDo;

		[Input("Enabled", IsSingle=true)]
		private IDiffSpread<bool> FInEnabled;

		[Output("Corners RGB Map")]
		private ISpread<Vector2D> FOutCornersRGBMap;

		[Output("Corners Depth Map")]
		private ISpread<Vector2D> FOutCornersDepthMap;

		[Output("Corners Depth")]
		private ISpread<Double> FOutCornersDepth;

		[Output("Corners World")]
		private ISpread<Vector3D> FOutCornersXYZ;

		[Output("Status")]
		private ISpread<string> FOutStatus;

		private bool FInvalidateConnect = false;
		private bool FInvalidate = true;

		private KinectRuntime FRuntime;

		private byte[] FDataRGB = null;
		private short[] FDataDepth = null;
		private int[] FMappingColorToDepth = null;
		private CVImage FImageRGB = new CVImage();
		private CVImage FImageLuminance = new CVImage();
		private ColorImagePoint[] FMappingDepthToColor = null;

		private DepthImageFormat FDepthFormat;
		private ColorImageFormat FColorFormat;
		private Size FColorSize;
		private Size FDepthSize;

		private Object FLockRGB = new Object();
		private Object FLockDepth = new Object();

		#endregion

		[ImportingConstructor()]
		public FindBoardNode(IPluginHost host)
		{
		}

		public void Evaluate(int SpreadMax)
		{

			if (this.FInvalidateConnect)
			{
				if (FRuntime != null)
				{
					//remove any listeners
					FRuntime.ColorFrameReady -= FRuntime_ColorFrameReady;
					FRuntime.DepthFrameReady -= FRuntime_DepthFrameReady;
				}

				if (this.FInRuntime.PluginIO.IsConnected)
				{
					//Cache runtime node
					this.FRuntime = this.FInRuntime[0];

					if (FRuntime != null)
					{
						//add any listeners
						FRuntime.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(FRuntime_ColorFrameReady);
						FRuntime.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(FRuntime_DepthFrameReady);
					}

				}

				this.FInvalidateConnect = false;
			}

			try
			{
				if (FInDo[0])
				{
					FindBoards();
					FOutStatus[0] = "OK";
				}
			}
			catch (Exception e)
			{
				FOutStatus[0] = e.Message;
			}
		}

		void FRuntime_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
		{
			var frame = e.OpenDepthImageFrame();

			if (frame != null && FInEnabled[0])
			{
				lock (FLockDepth)
				{
					FDepthFormat = frame.Format;

					if (FDataDepth == null || FDataDepth.Length != frame.PixelDataLength)
					{
						FDataDepth = new short[frame.PixelDataLength];
						FDepthSize = new Size(frame.Width, frame.Height);
						FMappingDepthToColor = new ColorImagePoint[frame.Width * frame.Height];
					}

					frame.CopyPixelDataTo(FDataDepth);

					if (FColorFormat != ColorImageFormat.Undefined)
					{
						FRuntime.Runtime.MapDepthFrameToColorFrame(frame.Format, FDataDepth, FColorFormat, FMappingDepthToColor);
					}

				}
				frame.Dispose();
			}
		}

		void FRuntime_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
		{
			var frame = e.OpenColorImageFrame();
			
			if (frame != null && FInEnabled[0])
			{
				lock (FLockRGB)
				{
					FColorFormat = frame.Format;

					if (FDataRGB == null || FDataRGB.Length != frame.PixelDataLength)
					{
						FDataRGB = new byte[frame.PixelDataLength];
						FColorSize = new Size(frame.Width, frame.Height);
					}

					frame.CopyPixelDataTo(FDataRGB);

				}
				 frame.Dispose();
			}
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


		private void FindBoards()
		{
			if (FDataRGB == null)
				throw (new Exception("Couldn't read RGB frame"));
			if (FDataDepth == null)
				throw (new Exception("Couldn't read depth frame")); 
			
			lock (FLockDepth)
			{
				lock (FLockRGB)
				{
					var kinect = FRuntime.Runtime;
					
					//allocate our version of the color image
					if (FImageRGB.Width != FColorSize.Width || FImageRGB.Height != FColorSize.Height)
					{
						var size = new Size(FColorSize.Width, FColorSize.Height);
						FImageRGB.Initialise(size, TColorFormat.RGBA8);
						FImageLuminance.Initialise(size, TColorFormat.L8);
					}

					//invert lookup table
					invertLookup();

					//convert rgb to luminance
					FImageRGB.SetPixels(FDataRGB);
					FImageRGB.GetImage(TColorFormat.L8, FImageLuminance);
					 
					//find chessboard corners
					var pointsInRGB = CameraCalibration.FindChessboardCorners(FImageLuminance.GetImage() as Image<Gray, byte>, new Size(FInBoardX[0], FInBoardY[0]), CALIB_CB_TYPE.ADAPTIVE_THRESH);

					//if we didn't find anything, then complain and quit
					if (pointsInRGB == null)
						throw(new Exception("No chessboard found."));

					FOutCornersRGBMap.SliceCount = pointsInRGB.Length;
					FOutCornersDepthMap.SliceCount = pointsInRGB.Length;
					FOutCornersDepth.SliceCount = pointsInRGB.Length;
					FOutCornersXYZ.SliceCount = pointsInRGB.Length;

					for(int i=0; i<pointsInRGB.Length; i++)
					{
						FOutCornersRGBMap[i] = new Vector2D((double)pointsInRGB[i].X, (double)pointsInRGB[i].Y);

						var xRGB = (int) pointsInRGB[i].X;
						var yRGB = (int) pointsInRGB[i].Y;

						var colorIndex = xRGB + yRGB * kinect.ColorStream.FrameWidth;
						var depthIndex = FMappingColorToDepth[colorIndex];
						var xDepth = depthIndex % kinect.DepthStream.FrameWidth;
						var yDepth = depthIndex / kinect.DepthStream.FrameWidth;

						FOutCornersDepthMap[i] = new Vector2D((double) xDepth, (double) yDepth);
						FOutCornersDepth[i] = (Double)FDataDepth[xDepth + yDepth * FDepthSize.Width];

						var world = kinect.MapDepthToSkeletonPoint(FDepthFormat, xRGB, yRGB, FDataDepth[depthIndex]);

						FOutCornersXYZ[i] = new Vector3D((double)world.X, (double)world.Y, (double)world.Z);
					}

				}
			}

			#region oldcode
			////if we've got matching depth and rgb frames then let's find some chessboards!
			////if (true || this.rgbFrameAvailable == this.depthFrameAvailable)
			//{
			//    //find points in rgb
			//    PointF[] pointsRGB;
			//    lock (FLockRGB)
			//    {
			//        var image = FImageLuminance.GetImage() as Emgu.CV.Image<Gray, byte>;
			//        pointsRGB = CameraCalibration.FindChessboardCorners(image, FBoardSize, CALIB_CB_TYPE.ADAPTIVE_THRESH);
			//    }

			//    if (pointsRGB == null)
			//        return;

			//    var frameDepth = FRuntime.Runtime.DepthStream.OpenNextFrame()
			//    //allocate output data
			//    PointF[] pointsDepthMap = new PointF[pointsRGB.Length];
			//    Vector3D[] pointsWorld = new Vector3D[pointsRGB.Length];
			//    ushort[] pointsDepthValue = new ushort[pointsRGB.Length];

			//    //build rgb->depth lookup table
			//    invertLookup();

			//    //lookup world xyz 
			//    lock (FLockDepth)
			//    {
			//        int idxRGB, idxDepth;
			//        ushort depth;

			//        var widthRGB = this.FRuntime.Runtime.ColorStream.FrameWidth;
			//        var heightRGB = this.FRuntime.Runtime.ColorStream.FrameHeight;
			//        var widthDepth = this.FRuntime.Runtime.DepthStream.FrameWidth;
			//        var formatDepth = this.FRuntime.Runtime.DepthStream.Format;

			//        for (int i = 0; i < pointsRGB.Length; i++)
			//        {
			//            var xInRGB = pointsRGB[i].X;
			//            var yInRGB = pointsRGB[i].Y;

			//            if (xInRGB < 0 || xInRGB >= widthRGB || yInRGB < 0 || yInRGB >= heightRGB)
			//            {
			//                //this corner appears to be outside rgb map, skip it
			//                continue;
			//            }

			//            idxRGB = (int) xInRGB + (int)( (int) yInRGB * widthRGB);
			//            idxDepth = FMappingColorToDepth[idxRGB];

			//            pointsDepthMap[i] = new PointF(idxDepth % widthDepth, idxDepth / widthDepth);
			//            depth = (ushort) FDataDepth[idxDepth];
			//            var world = FRuntime.Runtime.MapDepthToSkeletonPoint(formatDepth, (int) pointsDepthMap[i].X, (int) pointsDepthMap[i].Y, FDataDepth[idxDepth]);

			//            pointsWorld[i] = new Vector3D((double) world.X, (double) world.Y, (double) world.Z);
			//        }
			//    }

			//    lock (FLockPoints)
			//    {
			//        FPointsRGBMap.SliceCount = pointsRGB.Length;
			//        FPointsDepthMap.SliceCount = pointsRGB.Length;
			//        FPointsWorld.SliceCount = pointsRGB.Length;

			//        for (int i = 0; i < pointsRGB.Length; i++)
			//        {
			//            FPointsRGBMap[i] = new Vector2D((double) pointsRGB[i].X, (double) pointsRGB[i].Y);
			//            FPointsDepthMap[i] = new Vector2D((double)pointsDepthMap[i].X, (double)pointsDepthMap[i].Y);
			//            FPointsWorld[i] = pointsWorld[i];
			//        }
			//    }

			// }
			#endregion
		}

		private void invertLookup()
		{
			if (FMappingColorToDepth == null || FMappingColorToDepth.Length != FRuntime.Runtime.ColorStream.FramePixelDataLength)
				FMappingColorToDepth = new int[FRuntime.Runtime.ColorStream.FramePixelDataLength];

			int stride = FRuntime.Runtime.ColorStream.FrameWidth;

			for (int i = 0; i < FMappingDepthToColor.Length; i++)
			{
				var depthToColor = FMappingDepthToColor[i];
				FMappingColorToDepth[depthToColor.X + stride * depthToColor.Y] = i;
			}
		}
	}
}
