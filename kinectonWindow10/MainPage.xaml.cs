
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using WindowsPreview.Kinect;
using Windows.UI.Xaml.Shapes;
using Windows.UI;
using System.Xml.Linq;
using System.Linq;
using System.IO;
using Windows.Storage;
using Accord.Statistics.Models.Markov.Topology;
using Accord.Statistics.Models.Markov;
using Accord.Statistics.Models.Markov.Learning;
using Windows.UI.Popups;
using System.Threading.Tasks;
using System.Threading;
using Windows.Foundation;
using System.ComponentModel;


// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=234238

namespace kinectonWindow10
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {

        public MainPage()
        {
            this.InitializeComponent();
            this.Loaded += MainPage_Loaded;
        }
        //initialize varaibles and namespaces
        jiontAngle angle;
        Line line;
        DepthSpacePoint[] DepthSpacePointArray;
        Joint[] JointArray;
        KinectSensor sensor;
        InfraredFrameReader irReader;
        ushort[] irData;
        byte[] irDataConverted;
        WriteableBitmap irBatmap;
        Body[] bodies;
        MultiSourceFrameReader msfr;
        List<double[]> leftHandArray;
        List<double> leftHandVectorXYZ;
        List<double> rightHandVectorXYZ;
        List<double> leftHandVectorX;
        List<double> leftHandVectorY;
        List<double> leftHandVectorZ;
        List<double> leftHandVectorXY;
        List<double> leftHandVectorYZ;
        List<double> leftHandVectorXZ;
        bool flag = false;//open close hand flag
        int frameCount = 0;
        double handShoulderDistance = 0;
        double leftElbowAngle;
        double rightElbowAngle;
        double leftShoulderAngle;
        double rightShoulderAngle;
        int[][] inputSequences;
        bool startProcessing = false;
        double l1 = 0, l2 = 0, l3 = 0, R1 = 0, R2 = 0, R3 = 0;
        int same = 0;
        void MainPage_Loaded(object sender, RoutedEventArgs e)
        {
            //this.Width = Window.Current.CoreWindow.Bounds.Width;
            //this.Height = Window.Current.CoreWindow.Bounds.Height;
            //assign variables and namespaces 
            sensor = KinectSensor.GetDefault();
            irReader = sensor.InfraredFrameSource.OpenReader();
            FrameDescription fd = sensor.InfraredFrameSource.FrameDescription;
            irData = new ushort[fd.LengthInPixels];
            irDataConverted = new byte[fd.LengthInPixels * 4];
            irBatmap = new WriteableBitmap(fd.Width, fd.Height);
            image.Source = irBatmap;
            bodies = new Body[6];
            msfr = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.Infrared);
            msfr.MultiSourceFrameArrived += Msfr_MultiSourceFrameArrived;

            //bones.Stroke = new SolidColorBrush(Colors.Red);
            //bones.StrokeThickness = 5;
            sensor.Open();
            // irReader.FrameArrived += irReader_FrameArrived;
            //elps = new Ellipse[25];
            //SaveXml();
            //gestureVerify();
            leftHandArray = new List<double[]>();
            leftHandVectorXYZ = new List<double>();
            rightHandVectorXYZ = new List<double>();
            leftHandVectorX = new List<double>();
            leftHandVectorY = new List<double>();
            leftHandVectorZ = new List<double>();
            leftHandVectorXY = new List<double>();
            leftHandVectorYZ = new List<double>();
            leftHandVectorXZ = new List<double>();
            leftHandCanvas.Children.Clear();
            righttHandCanvas.Children.Clear();
        }
        private async void btnStartprocessing_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new Windows.UI.Popups.MessageDialog("Do you want to start processing","Start");
            dlg.Commands.Add(new UICommand("Add to database", null, "add"));
            dlg.Commands.Add(new UICommand("Gesture verify", null, "test"));            
            var result = await dlg.ShowAsync();
            if (result.Id == "add")
            {
                flag = true;
            }
            else
            {
                flag = false;
            }
            IAsyncOperation<IUICommand> asyncCommand = null;
            var wait = new Windows.UI.Popups.MessageDialog("Please be ready to start the processing","Wait 5 seconds");
            asyncCommand = wait.ShowAsync();
            if (asyncCommand != null)
            {
                await Task.Delay(5000);
                asyncCommand.Cancel();
                asyncCommand = null;
            }
            startProcessing = true;
        }
        private async void SaveXml()
        {
            signName.IsOpen = true;
            //var dlg = new Windows.UI.Popups.MessageDialog("test","Add word");
            //dlg.Commands.Add(new UICommand("Add to database", null, "add"));
            //dlg.Commands.Add(new UICommand("Gesture verify", null, "test"));

            //dlg.ShowAsync();
            //string XML_DATA_FILE = "gesture.xml";
            //string customerXMLPath = System.IO.Path.Combine(ApplicationData.Current.LocalFolder.Path, XML_DATA_FILE);

            ////string FILE_PATH = "/gesture.xml";
            //XDocument xmlDoc = XDocument.Load(@"gesture.xml");
            ////XDocument xmlDoc = XDocument.Load(customerXMLPath);
            //var count = xmlDoc.Descendants("gesture").Count();
            //xmlDoc.Root.Add(new XElement("gesture",
            //                 new XElement("id", count + 1),
            //                 new XElement("path", "mole"), 
            //                 new XElement("reference", 12)
            //            ));
            //StorageFolder storageFolder = ApplicationData.Current.LocalFolder;
            //var newfile = await storageFolder.CreateFileAsync(XML_DATA_FILE,
            //CreationCollisionOption.ReplaceExisting);
            //await FileIO.WriteTextAsync(newfile, xmlDoc.ToString());
        }

        private void Msfr_MultiSourceFrameArrived(MultiSourceFrameReader sender, MultiSourceFrameArrivedEventArgs args)
        {
            using (MultiSourceFrame msf = args.FrameReference.AcquireFrame())
            {
                if (msf != null)
                {
                    using (BodyFrame bodyFrame = msf.BodyFrameReference.AcquireFrame())
                    {
                        using (InfraredFrame irFrame = msf.InfraredFrameReference.AcquireFrame())
                        {
                            if (bodyFrame != null && irFrame != null)
                            {

                                irFrame.CopyFrameDataToArray(irData);
                                for (int i = 0; i < irData.Length; i++)
                                {
                                    byte intensity = (byte)(irData[i] >> 8);
                                    irDataConverted[i * 4] = intensity;
                                    irDataConverted[i * 4 + 1] = intensity;
                                    irDataConverted[i * 4 + 2] = intensity;
                                    irDataConverted[i * 4 + 3] = 255;
                                }
                                irDataConverted.CopyTo(irBatmap.PixelBuffer);
                                irBatmap.Invalidate();

                                bodyFrame.GetAndRefreshBodyData(bodies);
                                bodyCanvas.Children.Clear();
                                foreach (Body body in bodies)
                                {
                                    if (body.IsTracked &&  body != null)
                                    {
                                        //25 points and skeleton
                                        this.bodySkeleton(body);
                                        //angle between joints
                                        this.jointsAngle(body);
                                        //hand position x,y,z points
                                        this.handPosition(body);
                                        //hand direction,hand move, away or toward the body
                                        this.handDirection(body);
                                    }
                                }
                            }
                        }
                    }
                }
            }

        }
        //find the angle
        void jointsAngle(Body body)
        {
            this.jointAndJointsPoint(body);
            angle = new jiontAngle();
            //left elbow angle
            leftElbowAngle = angle.AngleBetweenJoints(JointArray[4], JointArray[5], JointArray[6]);
            //left wrist angle
            double leftWristAngle = angle.AngleBetweenJoints(JointArray[5], JointArray[6], JointArray[7]);
            //right elbow angle
            rightElbowAngle = angle.AngleBetweenJoints(JointArray[8], JointArray[9], JointArray[10]);
            //right wrist angle
            double rigthWristAngle = angle.AngleBetweenJoints(JointArray[9], JointArray[10], JointArray[11]);
            //shoulder angles
            leftShoulderAngle = angle.AngleBetweenJoints(JointArray[20], JointArray[4], JointArray[5]);
            rightShoulderAngle = angle.AngleBetweenJoints(JointArray[20], JointArray[8], JointArray[9]);
            TextBlock anglval;
            if (leftElbowAngle > 0)
            {
                anglval = new TextBlock { Text = Math.Floor(leftElbowAngle).ToString(), FontSize = 10 };
                if (anglval != null)
                {
                    bodyCanvas.Children.Add(anglval);
                    Canvas.SetLeft(anglval, DepthSpacePointArray[5].X - 3);
                    Canvas.SetTop(anglval, DepthSpacePointArray[5].Y - 3);
                }
            }
            TextBlock anglvalright;
            if (rightElbowAngle > 0)
            {
                anglvalright = new TextBlock { Text = Math.Floor(rightElbowAngle).ToString(), FontSize = 10 };
                if (anglvalright != null)
                {
                    bodyCanvas.Children.Add(anglvalright);
                    Canvas.SetLeft(anglvalright, DepthSpacePointArray[9].X - 3);
                    Canvas.SetTop(anglvalright, DepthSpacePointArray[9].Y - 3);
                }
            }
            //left shoulder angle
            TextBlock angleftShoulder;
            if (leftShoulderAngle > 0)
            {
                angleftShoulder = new TextBlock { Text = Math.Floor(leftShoulderAngle).ToString(), FontSize = 10 };
                if (angleftShoulder != null)
                {
                    bodyCanvas.Children.Add(angleftShoulder);
                    Canvas.SetLeft(angleftShoulder, DepthSpacePointArray[2].X -3);
                    Canvas.SetTop(angleftShoulder, DepthSpacePointArray[2].Y -3);
                }
            }
            TextBlock angRightShoulder;
            if (rightShoulderAngle > 0)
            {
                angRightShoulder = new TextBlock { Text =Math.Floor(rightShoulderAngle).ToString(),FontSize=10 };
                if (angRightShoulder != null)
                {
                    bodyCanvas.Children.Add(angRightShoulder);
                    Canvas.SetLeft(angRightShoulder, DepthSpacePointArray[8].X - 3);
                    Canvas.SetTop(angRightShoulder, DepthSpacePointArray[8].Y - 3);
                }
            }
        }

        // hand position
        void handPosition(Body body)
        {
            if (startProcessing)
            {
                leftDepthPoint.Text = "Left shoulder angle: " + leftShoulderAngle;
                rightDepthPoint.Text = "right shuoulder angle: " + rightShoulderAngle;

                frameCount += 1;
                Joint HandRight = body.Joints[JointType.HandRight];
                Joint HandLeft = body.Joints[JointType.HandLeft];
                Joint ShoulderLeft = body.Joints[JointType.ShoulderLeft];
                int secondFrame = frameCount % 2;
                bool continues = false;
                //frame value should be > 5 and also take every 5th frame to normalize our data

                if (frameCount > 2 && secondFrame == 0 && !continues)
                {
                    //long Timestamp;
                    //velocity distance/timestamp
                    //double distance = Math.Sqrt(Math.Pow(xx, 2) + Math.Pow(yy, 2) + Math.Pow(zz, 2));
                    double[] d3arrayLeft = new double[3] { HandLeft.Position.X, HandLeft.Position.Y, HandLeft.Position.Z };
                    double[] d3arrayRight = new double[3] { HandRight.Position.X, HandRight.Position.Y, HandRight.Position.Z };
                    leftHandArray.Add(d3arrayLeft);
                    //make two vectors one is from x,y,z.x, y,z and other like xxxxxx,yyyyy,zzzzz
                    //do for VECTORS x,then y,then z,then xy,xz,yz, and then xyz
                    //x,y,z
                    // if not same frame
                    //left hand array
                    double first = Math.Abs(Convert.ToInt32(d3arrayLeft[0] * 10));
                    double second = Math.Abs(Convert.ToInt32(d3arrayLeft[1] * 10));
                    double third = Math.Abs(Convert.ToInt32(d3arrayLeft[2] * 10));
                    double fourth = Math.Abs(Convert.ToInt32(d3arrayRight[0] * 10));
                    double fifth = Math.Abs(Convert.ToInt32(d3arrayRight[1] * 10));
                    double sixth = Math.Abs(Convert.ToInt32(d3arrayRight[2] * 10));
                    if (first != l1 || second != l2 || third != l3 || R1 != fourth || R2 != fifth || R3 != sixth)
                    {
                        leftHandVectorXYZ.Add(first);
                        leftHandVectorXYZ.Add(second);
                        leftHandVectorXYZ.Add(third);
                        rightHandVectorXYZ.Add(fourth);
                        rightHandVectorXYZ.Add(fifth);
                        rightHandVectorXYZ.Add(sixth);
                        same = 0;
                    }
                    else {
                        same += 1;
                        if (same > 6 && leftHandVectorXYZ.Count > 20 && rightHandVectorXYZ.Count > 20)
                        {
                            continues = true;
                        }
                    }
                    l1 = first; l2 = second; l3 = third; R1 = fourth; R2 = fifth; R3 = sixth;

                    //right hand array

                    //first = Math.Abs(Convert.ToInt32(rightHandVectorXYZ[0] * 10));
                    //second = Math.Abs(Convert.ToInt32(rightHandVectorXYZ[1] * 10));
                    //third = Math.Abs(Convert.ToInt32(rightHandVectorXYZ[2] * 10));
                    //if (first != R1 || second != R2 || third != R3)
                    //{
                    //    rightHandVectorXYZ.Add(first);
                    //    rightHandVectorXYZ.Add(second);
                    //    rightHandVectorXYZ.Add(third);
                    //    same = 0;
                    //}
                    //else {
                    //    same += 1;
                    //    if (same > 20 && rightHandVectorXYZ.Count > 27)
                    //    {
                    //        continues = true;
                    //    }
                    //}
                    //R1 = first; R2 = second; R3 = third;


                    DepthSpacePoint HandLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandLeft.Position);
                    Ellipse eps = new Ellipse()
                    {
                        Width = 10,
                        Height = 10,
                        Fill = new SolidColorBrush(Color.FromArgb(255, 0, 255, 0))
                    };
                    // left trajectory
                    leftHandCanvas.Children.Add(eps);
                    Canvas.SetLeft(eps, HandLeftPosition.X);
                    Canvas.SetTop(eps, HandLeftPosition.Y);
                    Canvas.SetZIndex(eps, Convert.ToInt32(Math.Ceiling(HandLeft.Position.Z)));

                    DepthSpacePoint HandRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandRight.Position);
                    Ellipse epsr = new Ellipse()
                    {
                        Width = 10,
                        Height = 10,
                        Fill = new SolidColorBrush(Color.FromArgb(255, 0, 255, 0))
                    };
                    righttHandCanvas.Children.Add(epsr);
                    Canvas.SetLeft(epsr, HandRightPosition.X);
                    Canvas.SetTop(epsr, HandRightPosition.Y);
                    Canvas.SetZIndex(epsr, Convert.ToInt32(Math.Ceiling(HandRight.Position.Z)));

                    //if (handShoulderDistance == 0)
                    //{
                    //    double xx = HandLeft.Position.X - ShoulderLeft.Position.X;
                    //    double yy = HandLeft.Position.Y - ShoulderLeft.Position.Y;
                    //    double zz = HandLeft.Position.Z - ShoulderLeft.Position.Z;
                    //    handShoulderDistance = Math.Sqrt(Math.Pow(xx, 2) + Math.Pow(yy, 2) + Math.Pow(zz, 2));
                    //}
                }
                if (continues)
                {
                    leftHandVectorXYZ = angle.StatisticalOutLierAnalysis(leftHandVectorXYZ);
                    leftHandVectorXYZ = angle.StatisticalOutLierAnalysis(leftHandVectorXYZ);
                    startProcessing = false;
                    if (flag) //if add new gesture
                    {
                        leftHandCanvas.Children.Clear();
                        righttHandCanvas.Children.Clear();
                        signName.IsOpen = true;
                        //SavePath();
                    }
                    else
                    {
                        leftHandCanvas.Children.Clear();
                        righttHandCanvas.Children.Clear();                        
                        gestureVerify();
                    }
                }
            }
        }
        //reference value pass 0,1,2,3,4,5,6
        private void referenceKey_Click(object sender, RoutedEventArgs e)
        {
            if (referenceClass.Text != null && referenceClass.Text != "")
            {
                int refval = Convert.ToInt32(referenceClass.Text);
                signName.IsOpen = false;
                SavePath(refval);
            }
        }
        //save data to
        private async void SavePath(int refval)
        {
            string _XYZleft = string.Join(",", leftHandVectorXYZ);
            string _XYZright = string.Join(",", rightHandVectorXYZ);
            //both left and right
            string _XYZ = _XYZleft + _XYZright;
            string _X = string.Join(",", leftHandVectorX);
            string _Y = string.Join(",", leftHandVectorY);
            string _Z = string.Join(",", leftHandVectorZ);

            string _XY = string.Join(",", leftHandVectorXY);
            string _YZ = string.Join(",", leftHandVectorYZ);

            string _XZ = string.Join(",", leftHandVectorXZ);
            string _XXYYZZ = _X + "," + _Y + "," + _Z;//xxxxxxxyyyyyzzzzz(first all x,then all y then all z)

            object val = "value of XYZ left: " + _XYZleft + " value of X: " + _X + " value of Y: " + _Y + " value of Z: " + _Z
                + " value of XY: " + _XY + " value of YZ: " + _YZ + " value of XZ: " + _XZ + " value of XXYYZZ: " + _XXYYZZ;
            //ClearValue all value and canvas
            //write test code here

            string XML_DATA_FILE = "gesture.xml";
            string customerXMLPath = System.IO.Path.Combine(ApplicationData.Current.LocalFolder.Path, XML_DATA_FILE);
            
            XDocument xmlDoc = XDocument.Load(customerXMLPath);
           // XDocument xmlDoc = XDocument.Load(@"gesture.xml");
            var count = xmlDoc.Descendants("gesture").Count();
            xmlDoc.Root.Add(new XElement("gesture",
                             new XElement("id", count + 1),
                             new XElement("path", _XYZ),
                             new XElement("reference", refval)
                        ));
            StorageFolder storageFolder = ApplicationData.Current.LocalFolder;
            var newfile = await storageFolder.CreateFileAsync(XML_DATA_FILE,
            CreationCollisionOption.ReplaceExisting);
            await FileIO.WriteTextAsync(newfile, xmlDoc.ToString());

            leftHandArray.Clear();
            leftHandVectorXYZ.Clear();
            leftHandVectorXYZ.Clear();
            leftHandVectorX.Clear();
            leftHandVectorY.Clear();
            leftHandVectorZ.Clear();
            leftHandVectorXY.Clear();
            leftHandVectorYZ.Clear();
            rightHandVectorXYZ.Clear();
            
            //flag = false;
            handShoulderDistance = 0;
            startProcessing = false;
            same = 0;
        }
        //verify data
        private async void gestureVerify()
        {
            IAsyncOperation<IUICommand> asyncCommand1 = null;
            var wait1 = new Windows.UI.Popups.MessageDialog("Please Wait, Verification has been started!");
            asyncCommand1 = wait1.ShowAsync();
            if (asyncCommand1 != null)
            {
                await Task.Delay(1000);
                asyncCommand1.Cancel();
                asyncCommand1 = null;
            }

            string _XYZleft = string.Join(",", leftHandVectorXYZ);
            string _XYZright = string.Join(",", rightHandVectorXYZ);
            //both left and right
            string _XYZ = _XYZleft + _XYZright;
            string _X = string.Join(",", leftHandVectorX);
            string _Y = string.Join(",", leftHandVectorY);
            string _Z = string.Join(",", leftHandVectorZ);

            string _XY = string.Join(",", leftHandVectorXY);
            string _YZ = string.Join(",", leftHandVectorYZ);

            string _XZ = string.Join(",", leftHandVectorXZ);
            string _XXYYZZ = _X + "," + _Y + "," + _Z;//xxxxxxxyyyyyzzzzz(first all x,then all y then all z)

            object val = "value of XYZ left: " + _XYZleft + " value of X: " + _X + " value of Y: " + _Y + " value of Z: " + _Z
                + " value of XY: " + _XY + " value of YZ: " + _YZ + " value of XZ: " + _XZ + " value of XXYYZZ: " + _XXYYZZ;
            //ClearValue all value and canvas
            //write test code here

            //string XML_DATA_FILE = "gesture.xml";
            // string customerXMLPath = System.IO.Path.Combine(ApplicationData.Current.LocalFolder.Path, XML_DATA_FILE);
            //XDocument xmlDoc = XDocument.Load(customerXMLPath);
            XDocument xmlDoc = XDocument.Load(@"gesture.xml");
            var count = xmlDoc.Descendants("gesture").Count();

            //load data
            int ct = 0;
            int maxArrayVal = 0;
            inputSequences = new int[count][];
            int[] outputLabels = new int[count];
            foreach (XElement gstr in xmlDoc.Root.Nodes())
            {
                string gestur = gstr.Element("path").Value;
                int[] nums = gestur.Split(',').Select(int.Parse).ToArray();
                inputSequences[ct] = nums;
                if(nums.Length > maxArrayVal)
                {
                    maxArrayVal = nums.Length;
                }
                outputLabels[ct] = Convert.ToInt32( gstr.Element("reference").Value);
                ct += 1;
            }
            // Now consider their respective class labels
            // int[] outputLabels = { 0,0,0,0,1,1,1,1,2,2,2,2};

            // We will use a single topology for all inner models, but we 
            // could also have explicitled different topologies for each:
            // two topology. 1.Ergodic 2.Forward
            //i am usning forward
            int classesLength = outputLabels.Length + 1;// or any things
            ITopology forward = new Forward(states:5, deepness: 5);

            // Now we create a hidden Markov classifier with the given topology
            HiddenMarkovClassifier classifier = new HiddenMarkovClassifier(classes: classesLength,
                topology: forward, symbols: maxArrayVal);//maximum count

            // And create a algorithms to teach each of the inner models
            var teacher = new HiddenMarkovClassifierLearning(classifier,

                // We can specify individual training options for each inner model:
                modelIndex => new BaumWelchLearning(classifier.Models[modelIndex])
                {
                    Tolerance = 0.01, // iterate until loglikelihood changes less than 0.001
                    Iterations = 0     // don't place an upper limit on the number of iterations
                });
            //uncomment and comment for testing
            while (true)
            {
                //double error = teacher.ComputeError(inputSequences, outputLabels);
               // var resultWord3 = new Windows.UI.Popups.MessageDialog("verification is in process please wait!");
              //  resultWord3.ShowAsync();
                double error = teacher.Run(inputSequences, outputLabels);
                if (error < 0.001) break;
            }




            // After training has finished, we can check the 
            // output classificaton label for some sequences. 

            int[] arrayword = _XYZ.Split(',').Select(int.Parse).ToArray();
            int word = classifier.Compute(arrayword);    // output  word = 0,2,3,4,5,6,7,8,9
            //int[] arrayword1 = new[] { 1, 4, 12, 1, 4, 12, 2, 2, 11, 3, 2, 12, 4, 2, 12, 3, 2, 12, 3, 1, 12, 2, 0, 11, 1, 2, 11, 3, 3, 11, 1, 2, 12, 1, 2, 12, 1, 2, 123, 4, 13, 4, 4, 13, 4, 2, 12, 5, 2, 13, 6, 2, 14, 6, 2, 14, 5, 2, 14, 4, 2, 13, 2, 1, 11, 1, 5, 12, 1, 5, 12, 1, 6, 12, 1, 6, 13 };
            //int word = classifier.Compute(arrayword1);    // output  word = 0,2,3,4,5,6,7,8,9

            XDocument signLoad = XDocument.Load(@"sign.xml");
            bool isExist = false;
            foreach (XElement sign in signLoad.Root.Nodes())
            {
                int label = Convert.ToInt32(sign.Element("label").Value);
                if (word == label)
                {
                    isExist = true;
                    string text = sign.Element("text").Value;
                    wordname.Text = text;
                    var resultWord = new Windows.UI.Popups.MessageDialog("The word is ##:: " + text, "Sign");
                    resultWord.ShowAsync();
                    break;
                }
                //else {
                //    var resultWord1 = new Windows.UI.Popups.MessageDialog("Result not found!");
                //    resultWord1.ShowAsync();
                //    break;
                //}
                }
            if (!isExist) //if word not exist
            {
                  var resultWord1 = new Windows.UI.Popups.MessageDialog("Result not found!");
                  resultWord1.ShowAsync();
            }
            leftHandArray.Clear();
            leftHandVectorXYZ.Clear();
            leftHandVectorXYZ.Clear();
            leftHandVectorX.Clear();
            leftHandVectorY.Clear();
            leftHandVectorZ.Clear();
            leftHandVectorXY.Clear();
            leftHandVectorYZ.Clear();
            rightHandVectorXYZ.Clear();
            //leftHandCanvas.Children.Clear();
            //righttHandCanvas.Children.Clear();
            //flag = false;
            handShoulderDistance = 0;
            //startProcessing = false;
            same = 0;
        }
        //hand direction,hand is moving from the body,or toward the body
        void handDirection(Body body)
        {
            Joint handLeft = body.Joints[JointType.HandLeft];
            Joint shoulderLeft = body.Joints[JointType.ShoulderLeft];
            double xx = handLeft.Position.X - shoulderLeft.Position.X;
            double yy = handLeft.Position.Y - shoulderLeft.Position.Y;
            double zz = handLeft.Position.Z - shoulderLeft.Position.Z;
            double distance = Math.Sqrt(Math.Pow(xx, 2) + Math.Pow(yy, 2) + Math.Pow(zz, 2));
            if (distance > handShoulderDistance)
            { distanceMovement.Text = "move away from the body"; }
            else { distanceMovement.Text = "move toward the body"; }
        }
        void bodySkeleton(Body body)
        {
            if (body != null)
            {
                double distances = 0;
                this.jointAndJointsPoint(body);
                //if (JointArray[0].TrackingState == TrackingState.Tracked)
                //{
                //    distances = angle.Length(JointArray[0].Position);
                //}
                //bodyDistance.Text = distances.ToString();
                Color jointColor;
                //if (distances > 3)
                //{
                //    jointColor = Color.FromArgb(255, 255, 255, 0);
                //}
                //if (distances < 1)
                //{
                //    jointColor = Color.FromArgb(155, 155, 155, 0);
                //}
                //else
                //{
                //    jointColor = Color.FromArgb(25, 25, 25, 0);
                //}
                jointColor = Color.FromArgb(155, 155, 155, 0);
                Ellipse[] ellipse = new Ellipse[25];

                for (int p = 0; p < DepthSpacePointArray.Length; p++)
                {
                    //ellipse on joints
                    ellipse[p] = new Ellipse()
                    {
                        Width = 10,
                        Height = 10,
                        Fill = new SolidColorBrush(jointColor)
                    };
                    bodyCanvas.Children.Add(ellipse[p]);
                    //bodyCanvas.Children.Add(eps[p]);
                    Canvas.SetLeft(ellipse[p], DepthSpacePointArray[p].X - 2);
                    Canvas.SetTop(ellipse[p], DepthSpacePointArray[p].Y - 2);
                }
            }
        }
        //joint and joint position and bones skeleton
        void jointAndJointsPoint(Body body)
        {
            if (body != null)
            {
                Joint SpineBase = body.Joints[JointType.SpineBase];
                Joint SpineMid = body.Joints[JointType.SpineMid];
                Joint Neck = body.Joints[JointType.Neck];
                Joint Head = body.Joints[JointType.Head];
                Joint ShoulderLeft = body.Joints[JointType.ShoulderLeft];
                Joint ElbowLeft = body.Joints[JointType.ElbowLeft];
                Joint WristLeft = body.Joints[JointType.WristLeft];
                Joint HandLeft = body.Joints[JointType.HandLeft];
                Joint ShoulderRight = body.Joints[JointType.ShoulderRight];
                Joint ElbowRight = body.Joints[JointType.ElbowRight];
                Joint WristRight = body.Joints[JointType.WristRight];
                Joint HandRight = body.Joints[JointType.HandRight];
                Joint HipLeft = body.Joints[JointType.HipLeft];
                Joint KneeLeft = body.Joints[JointType.KneeLeft];
                Joint AnkleLeft = body.Joints[JointType.AnkleLeft];
                Joint FootLeft = body.Joints[JointType.FootLeft];
                Joint HipRight = body.Joints[JointType.HipRight];
                Joint KneeRight = body.Joints[JointType.KneeRight];
                Joint AnkleRight = body.Joints[JointType.AnkleRight];
                Joint FootRight = body.Joints[JointType.FootRight];
                Joint SpineShoulder = body.Joints[JointType.SpineShoulder];
                Joint HandTipLeft = body.Joints[JointType.HandTipLeft];
                Joint ThumbLeft = body.Joints[JointType.ThumbLeft];
                Joint HandTipRight = body.Joints[JointType.HandTipRight];
                Joint ThumbRight = body.Joints[JointType.ThumbRight];
                JointArray = new Joint[] { SpineBase,SpineMid,Neck,Head,ShoulderLeft,ElbowLeft,WristLeft,HandLeft,
                             ShoulderRight,ElbowRight,WristRight,HandRight,HipLeft,KneeLeft,AnkleLeft,FootLeft,
                             HipRight,KneeRight,AnkleRight,FootRight,SpineShoulder,HandTipLeft,ThumbLeft,HandTipRight,ThumbRight};
                //if (JointArray[0].TrackingState == TrackingState.Tracked && JointArray[1].TrackingState == TrackingState.Tracked && JointArray[2].TrackingState == TrackingState.Tracked
                //    && JointArray[3].TrackingState == TrackingState.Tracked && JointArray[4].TrackingState == TrackingState.Tracked && JointArray[5].TrackingState == TrackingState.Tracked
                //    && JointArray[6].TrackingState == TrackingState.Tracked && JointArray[7].TrackingState == TrackingState.Tracked && JointArray[8].TrackingState == TrackingState.Tracked
                //    && JointArray[9].TrackingState == TrackingState.Tracked && JointArray[10].TrackingState == TrackingState.Tracked && JointArray[11].TrackingState == TrackingState.Tracked
                //    && JointArray[12].TrackingState == TrackingState.Tracked && JointArray[13].TrackingState == TrackingState.Tracked && JointArray[14].TrackingState == TrackingState.Tracked
                //    && JointArray[15].TrackingState == TrackingState.Tracked && JointArray[16].TrackingState == TrackingState.Tracked && JointArray[17].TrackingState == TrackingState.Tracked
                //    && JointArray[18].TrackingState == TrackingState.Tracked && JointArray[19].TrackingState == TrackingState.Tracked && JointArray[20].TrackingState == TrackingState.Tracked
                //    && JointArray[21].TrackingState == TrackingState.Tracked && JointArray[22].TrackingState == TrackingState.Tracked && JointArray[23].TrackingState == TrackingState.Tracked
                //    && JointArray[24].TrackingState == TrackingState.Tracked)
                //{
                DepthSpacePoint SpineBasePosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(SpineBase.Position);
                DepthSpacePoint SpineMidPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(SpineMid.Position);
                DepthSpacePoint NeckPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(Neck.Position);
                DepthSpacePoint HeadPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(Head.Position);
                DepthSpacePoint ShoulderLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ShoulderLeft.Position);
                DepthSpacePoint ElbowLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ElbowLeft.Position);
                DepthSpacePoint WristLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(WristLeft.Position);
                DepthSpacePoint HandLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandLeft.Position);
                DepthSpacePoint ShoulderRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ShoulderRight.Position);
                DepthSpacePoint ElbowRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ElbowRight.Position);
                DepthSpacePoint WristRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(WristRight.Position);
                DepthSpacePoint HandRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandRight.Position);
                DepthSpacePoint HipLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HipLeft.Position);
                DepthSpacePoint KneeLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(KneeLeft.Position);
                DepthSpacePoint AnkleLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(AnkleLeft.Position);
                DepthSpacePoint FootLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(FootLeft.Position);
                DepthSpacePoint HipRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HipRight.Position);
                DepthSpacePoint KneeRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(KneeRight.Position);
                DepthSpacePoint AnkleRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(AnkleRight.Position);
                DepthSpacePoint FootRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(FootRight.Position);
                DepthSpacePoint HandTipLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandTipLeft.Position);
                DepthSpacePoint ThumbLeftPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ThumbLeft.Position);
                DepthSpacePoint HandTipRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(HandTipRight.Position);
                DepthSpacePoint ThumbRightPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(ThumbRight.Position);
                DepthSpacePoint SpineShoulderPosition = sensor.CoordinateMapper.MapCameraPointToDepthSpace(SpineShoulder.Position);
                DepthSpacePointArray = new DepthSpacePoint[] { HeadPosition,NeckPosition,ShoulderLeftPosition,SpineMidPosition,SpineBasePosition,
                                   ElbowLeftPosition,WristLeftPosition,HandLeftPosition,ShoulderRightPosition,ElbowRightPosition,
                                   WristRightPosition,HandRightPosition,HipLeftPosition,KneeLeftPosition,AnkleLeftPosition,FootLeftPosition,
                                   HipRightPosition,KneeRightPosition,AnkleRightPosition,FootRightPosition,SpineShoulderPosition,ThumbRightPosition,HandTipLeftPosition,ThumbLeftPosition,HandTipRightPosition};

                //bones                
                //Line[] bones = new Line[25];
                // var count = HandLeft.TrackingState.;
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = HeadPosition.X;
                line.Y1 = HeadPosition.Y;
                line.X2 = NeckPosition.X;
                line.Y2 = NeckPosition.Y;
                bodyCanvas.Children.Add(line);

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = HeadPosition.X;
                line.Y1 = HeadPosition.Y;
                line.X2 = SpineShoulderPosition.X;
                line.Y2 = SpineShoulderPosition.Y;
                bodyCanvas.Children.Add(line);

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineMidPosition.X;
                line.Y1 = SpineMidPosition.Y;
                line.X2 = SpineShoulderPosition.X;
                line.Y2 = SpineShoulderPosition.Y;
                bodyCanvas.Children.Add(line);
                //      { JointType.SpineShoulder, JointType.SpineMid },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineMidPosition.X;
                line.Y1 = SpineMidPosition.Y;
                line.X2 = SpineBasePosition.X;
                line.Y2 = SpineBasePosition.Y;
                bodyCanvas.Children.Add(line);

                // //       { JointType.SpineMid, JointType.SpineBase },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineShoulderPosition.X;
                line.Y1 = SpineShoulderPosition.Y;
                line.X2 = ShoulderRightPosition.X;
                line.Y2 = ShoulderRightPosition.Y;
                bodyCanvas.Children.Add(line);

                ////        { JointType.SpineShoulder, JointType.ShoulderRight },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineShoulderPosition.X;
                line.Y1 = SpineShoulderPosition.Y;
                line.X2 = ShoulderLeftPosition.X;
                line.Y2 = ShoulderLeftPosition.Y;
                bodyCanvas.Children.Add(line);


                // //       { JointType.SpineShoulder, JointType.ShoulderLeft },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineBasePosition.X;
                line.Y1 = SpineBasePosition.Y;
                line.X2 = HipRightPosition.X;
                line.Y2 = HipRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //  //      { JointType.SpineBase, JointType.HipRight },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = SpineBasePosition.X;
                line.Y1 = SpineBasePosition.Y;
                line.X2 = HipLeftPosition.X;
                line.Y2 = HipLeftPosition.Y;
                bodyCanvas.Children.Add(line);

                //   //     { JointType.SpineBase, JointType.HipLeft },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = ShoulderRightPosition.X;
                line.Y1 = ShoulderRightPosition.Y;
                line.X2 = ElbowRightPosition.X;
                line.Y2 = ElbowRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //        // Right Arm
                //    //    { JointType.ShoulderRight, JointType.ElbowRight },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = WristRightPosition.X;
                line.Y1 = WristRightPosition.Y;
                line.X2 = ElbowRightPosition.X;
                line.Y2 = ElbowRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //    //    { JointType.ElbowRight, JointType.WristRight },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5; line.X1 = WristRightPosition.X;
                line.Y1 = WristRightPosition.Y;
                line.X2 = HandRightPosition.X;
                line.Y2 = HandRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //    //    { JointType.WristRight, JointType.HandRight },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = HandTipRightPosition.X;
                line.Y1 = HandTipRightPosition.Y;
                line.X2 = HandRightPosition.X;
                line.Y2 = HandRightPosition.Y;
                bodyCanvas.Children.Add(line);
                //   //     { JointType.HandRight, JointType.HandTipRight },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = WristRightPosition.X;
                line.Y1 = WristRightPosition.Y;
                line.X2 = ThumbRightPosition.X;
                line.Y2 = ThumbRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //  //      { JointType.WristRight, JointType.ThumbRight },

                //            // Left Arm
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = ShoulderLeftPosition.X;
                line.Y1 = ShoulderLeftPosition.Y;
                line.X2 = ElbowLeftPosition.X;
                line.Y2 = ElbowLeftPosition.Y;
                bodyCanvas.Children.Add(line);

                //      //  { JointType.ShoulderLeft, JointType.ElbowLeft },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = WristLeftPosition.X;
                line.Y1 = WristLeftPosition.Y;
                line.X2 = ElbowLeftPosition.X;
                line.Y2 = ElbowLeftPosition.Y;
                bodyCanvas.Children.Add(line);

                //   //     { JointType.ElbowLeft, JointType.WristLeft },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = WristLeftPosition.X;
                line.Y1 = WristLeftPosition.Y;
                line.X2 = HandLeftPosition.X;
                line.Y2 = HandLeftPosition.Y;
                bodyCanvas.Children.Add(line);
                //    //    { JointType.WristLeft, JointType.HandLeft },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = HandTipLeftPosition.X;
                line.Y1 = HandTipLeftPosition.Y;
                line.X2 = HandLeftPosition.X;
                line.Y2 = HandLeftPosition.Y;
                bodyCanvas.Children.Add(line);
                //   //     { JointType.HandLeft, JointType.HandTipLeft },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = WristLeftPosition.X;
                line.Y1 = WristLeftPosition.Y;
                line.X2 = ThumbLeftPosition.X;
                line.Y2 = ThumbLeftPosition.Y;
                bodyCanvas.Children.Add(line);
                //   //     { JointType.WristLeft, JointType.ThumbLeft },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = KneeRightPosition.X;
                line.Y1 = KneeRightPosition.Y;
                line.X2 = HipRightPosition.X;
                line.Y2 = HipRightPosition.Y;
                bodyCanvas.Children.Add(line);
                //        // Right Leg
                //    //    { JointType.HipRight, JointType.KneeRight },

                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = KneeRightPosition.X;
                line.Y1 = KneeRightPosition.Y;
                line.X2 = AnkleRightPosition.X;
                line.Y2 = AnkleRightPosition.Y;
                bodyCanvas.Children.Add(line);
                
                //     //   { JointType.KneeRight, JointType.AnkleRight },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = FootRightPosition.X;
                line.Y1 = FootRightPosition.Y;
                line.X2 = AnkleRightPosition.X;
                line.Y2 = AnkleRightPosition.Y;
                bodyCanvas.Children.Add(line);

                //     //   { JointType.AnkleRight, JointType.FootRight },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = HipLeftPosition.X;
                line.Y1 = HipLeftPosition.Y;
                line.X2 = KneeLeftPosition.X;
                line.Y2 = KneeLeftPosition.Y;
                bodyCanvas.Children.Add(line);
                //        // Left Leg
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = AnkleLeftPosition.X;
                line.Y1 = AnkleLeftPosition.Y;
                line.X2 = KneeLeftPosition.X;
                line.Y2 = KneeLeftPosition.Y;
                bodyCanvas.Children.Add(line);
                //     //   { JointType.KneeLeft, JointType.AnkleLeft },
                line = new Line();
                line.Stroke = new SolidColorBrush(Colors.Red);
                line.StrokeThickness = 5;
                line.X1 = AnkleLeftPosition.X;
                line.Y1 = AnkleLeftPosition.Y;
                line.X2 = FootLeftPosition.X;
                line.Y2 = FootLeftPosition.Y;
                bodyCanvas.Children.Add(line);
              
            }
            
        }
        //close window
        private void MainWindow_Closing(object sender, RoutedEventArgs e)
        {
            if (this.irReader != null)
            {
                // irReader is IDisposable
                this.irReader.Dispose();
                this.irReader = null;
            }

            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }
        private void btnCancel_Click(object sender, RoutedEventArgs e)
        {
            //add others logic laters
            loginToSign.IsOpen = false;
        }
        private void btnLogin_Click(object sender, RoutedEventArgs e)
        {
            string email = emailaddress.Text;
            string password = user_password.Password;
            if(email == "shah" && password == "shah")
            {
                loginToSign.IsOpen = false;
                login_error_message.Visibility = Visibility.Collapsed;
            } else { loginToSign.IsOpen = true; login_error_message.Visibility = Visibility.Visible; }
        }
        
        // }
        //}
        //void irReader_FrameArrived(InfraredFrameReader sender, InfraredFrameArrivedEventArgs args)
        //{
        //    using (InfraredFrame irFrame = args.FrameReference.AcquireFrame())
        //    {
        //        if (irFrame != null)
        //        {
        //            irFrame.CopyFrameDataToArray(irData);
        //            for (int i = 0; i < irData.Length; i++)
        //            {
        //                byte intensity = (byte)(irData[i] >> 8);
        //                irDataConverted[i * 4] = intensity;
        //                irDataConverted[i * 4 + 1] = intensity;
        //                irDataConverted[i * 4 + 2] = intensity;
        //                irDataConverted[i * 4 + 3] = 255;
        //            }
        //            irDataConverted.CopyTo(irBatmap.PixelBuffer);
        //            irBatmap.Invalidate();
        //        }
        //    }
        //}
    }
}
