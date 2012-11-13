using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using VVVV.MSKinect.Lib;
using VVVV.Utils.VMath;
using Microsoft.Kinect;
using VVVV.PluginInterfaces.V1;
using VVVV.Nodes.OpenCV;
using Emgu.CV;
using Emgu.CV.Structure;
using System.Drawing;
using Emgu.CV.CvEnum;

namespace VVVV.Nodes.MSKinect
{
	[PluginInfo(Name = "FindBoardInWorld", Category = "Devices", Version = "Microsoft, FullHouse", Author = "elliotwoods", AutoEvaluate = true)]
	public class FindBoardInWorld : IPluginEvaluate, IDisposable, IPluginConnections
	{
		[Input("Context", IsSingle = true)]
		Pin<FullHouseContext> FInFullHouseContext;

		[Input("Board Size X", IsSingle = true)]
		IDiffSpread<int> FInBoardX;

		[Input("Board Size Y", IsSingle = true)]
		IDiffSpread<int> FInBoardY;

		[Input("Do", IsSingle = true, IsBang=true)]
		IDiffSpread<bool> FInDo;

		[Output("Depth")]
		ISpread<Vector3D> FOutDepth;

		[Output("World")]
		ISpread<Vector3D> FOutWorld;

		FullHouseNode FContext;
		EventHandler OnUpdate;
		bool FContextDirty = false;
		List<Vector3D> FWorldCorners = new List<Vector3D>();
		List<Vector3D> FDepthCorners = new List<Vector3D>();

		bool FCornersDirty = false;

		FindBoardInWorld()
		{
			OnUpdate = new EventHandler(FContext_Update);
		}


		public void Evaluate(int SpreadMax)
		{
			if (FContextDirty)
			{
				if (FInFullHouseContext[0] == null)
					FInFullHouseContext = null;
				else
				{
					FContext = FInFullHouseContext[0].FullHouseNode;
				}
			}

			if (FInDo[0])
				FContext.Update += OnUpdate;

			if (FCornersDirty)
			{
				lock(FWorldCorners)
				{
					FOutWorld.SliceCount = FWorldCorners.Count;
					for (int i = 0; i < FWorldCorners.Count; i++)
						FOutWorld[i] = FWorldCorners[i];
				}

				lock (FDepthCorners)
				{
					FOutDepth.SliceCount = FDepthCorners.Count;
					for (int i = 0; i < FDepthCorners.Count; i++)
						FOutDepth[i] = FDepthCorners[i];
				}
			}
		}

		void FContext_Update(object sender, EventArgs e)
		{
			FContext.Update -= OnUpdate;

			var colorWidth = FContext.FSensor.ColorStream.FrameWidth;
			var colorHeight = FContext.FSensor.ColorStream.FrameHeight;
			var depthWidth = FContext.FSensor.DepthStream.FrameWidth;
			var depthHeight = FContext.FSensor.DepthStream.FrameHeight;

			//find board positions in world coordinates

			//get mapped color image
			CVImage Image = new CVImage();
			//it's mapped so it's in depth coords
			Image.Initialise(new System.Drawing.Size(depthWidth, depthHeight), TColorFormat.RGBA8);
			lock (FContext.ColorLock)
			{
				Image.SetPixels(FContext.ColorData);
			}

			//create grayscale
			CVImage Luminance = new CVImage();
			Luminance.Initialise(Image.Size, TColorFormat.L8);
			Image.GetImage(TColorFormat.L8, Luminance);

			//find corners in rgb
			var pointsInRGB = CameraCalibration.FindChessboardCorners(Luminance.GetImage() as Image<Gray, byte>, new Size(FInBoardX[0], FInBoardY[0]), CALIB_CB_TYPE.ADAPTIVE_THRESH);

			//destroy images
			Image.Dispose();
			Luminance.Dispose();

			//find corners in world
			lock (FContext.DepthLock)
			{
				FContext.FWorldLock.AcquireReaderLock(500);
				try
				{
					var world = FContext.WorldData;
					lock (FDepthCorners)
					{
						FDepthCorners.Clear();
						lock (FWorldCorners)
						{
							FWorldCorners.Clear();
							foreach (var rgbCorner in pointsInRGB)
							{
								int index = (int)rgbCorner.X + (int)rgbCorner.Y * depthWidth;

								FDepthCorners.Add(new Vector3D(
										rgbCorner.X,
										rgbCorner.Y,
										(double) FContext.DepthData[index] / 1000.0
									));
								FWorldCorners.Add(new Vector3D(
										world[index * 4],
										world[index * 4 + 1],
										world[index * 4 + 2]
									));
							}
						}
					}
					FCornersDirty = true;
				}
				catch
				{

				}
				finally
				{
					FContext.FWorldLock.ReleaseReaderLock();
				}
			}
		}

		public void Dispose()
		{
		}

		public void ConnectPin(IPluginIO pin)
		{
			if (pin == FInFullHouseContext.PluginIO)
				FContextDirty = true;
		}

		public void DisconnectPin(IPluginIO pin)
		{
			if (pin == FInFullHouseContext.PluginIO)
				FContextDirty = true;
		}
	}
}
